#include <Windows.h>
#include <NuiApi.h>
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Inicializar el sensor Kinect
    INuiSensor* sensor = nullptr;
    HRESULT hr = NuiCreateSensorByIndex(0, &sensor);
    if (FAILED(hr) || sensor == nullptr) {
        std::cerr << "No se pudo inicializar el sensor Kinect." << std::endl;
        return -1;
    }

    hr = sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH);
    if (FAILED(hr)) {
        std::cerr << "No se pudo inicializar el flujo de profundidad." << std::endl;
        sensor->Release();
        return -1;
    }

    // Abrir el flujo de datos de profundidad
    HANDLE depthStream = nullptr;
    hr = sensor->NuiImageStreamOpen(
        NUI_IMAGE_TYPE_DEPTH,
        NUI_IMAGE_RESOLUTION_640x480,
        0,
        2,
        nullptr,
        &depthStream
    );
    if (FAILED(hr)) {
        std::cerr << "No se pudo abrir el flujo de profundidad." << std::endl;
        sensor->NuiShutdown();
        sensor->Release();
        return -1;
    }

    const int width = 640;
    const int height = 480;

    // Rango de profundidad permitido (en mm)
    const USHORT minDepth = 700;   // Profundidad mínima
    const USHORT maxDepth = 1600; // Profundidad máxima

    // Niveles de elevación en mm
    const USHORT interval = 100; // Intervalo entre contornos

    while (true) {
        NUI_IMAGE_FRAME imageFrame;
        hr = sensor->NuiImageStreamGetNextFrame(depthStream, 0, &imageFrame);
        if (FAILED(hr)) {
            continue; // Intentar obtener el siguiente frame
        }

        INuiFrameTexture* texture = imageFrame.pFrameTexture;
        NUI_LOCKED_RECT lockedRect;
        texture->LockRect(0, &lockedRect, nullptr, 0);

        if (lockedRect.Pitch != 0) {
            USHORT* buffer = (USHORT*)lockedRect.pBits;
            cv::Mat depthImage(height, width, CV_8UC1, cv::Scalar(0)); // Imagen inicial en escala de grises
            cv::Mat colorImage(height, width, CV_8UC3, cv::Scalar(0, 0, 0)); // Imagen en color (BGR)

            // Factor de escala para convertir profundidad a colores (0-255)
            float scale = 255.0f / (maxDepth - minDepth);

            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    int index = x + y * width;
                    USHORT depth = buffer[index];
                    USHORT realDepth = depth & 0x0fff; // Extraer los 13 bits de profundidad

                    // Normalizar profundidad al rango 0-255
                    if (realDepth >= minDepth && realDepth <= maxDepth) {
                        BYTE intensity = static_cast<BYTE>((realDepth - minDepth) * scale);
                        depthImage.at<BYTE>(y, x) = intensity;
                    }
                }
            }

            // Aplicar un mapa de colores
            cv::applyColorMap(depthImage, colorImage, cv::COLORMAP_JET);

            // Generar contornos topográficos
            for (int i = minDepth; i <= maxDepth; i += interval) {
                cv::Mat mask = (depthImage >= ((i - minDepth) * scale / interval)) &
                    (depthImage < ((i - minDepth + interval) * scale / interval));

                // Detectar contornos en la máscara
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

                // Dibujar contornos sobre la imagen en color
                cv::drawContours(colorImage, contours, -1, cv::Scalar(0, 0, 0), 1);
            }

            // Mostrar la imagen con contornos
            cv::imshow("Mapa Topográfico con Contornos", colorImage);

            // Salir si se presiona 'Esc'
            if (cv::waitKey(1) == 27) {
                break;
            }
        }

        texture->UnlockRect(0);
        sensor->NuiImageStreamReleaseFrame(depthStream, &imageFrame);
    }

    // Liberar recursos
    sensor->NuiShutdown();
    sensor->Release();

    return 0;
}
