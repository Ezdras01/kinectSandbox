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
    const USHORT minDepth = 700;   // Ligeramente más bajo
    const USHORT maxDepth = 1600; // Ligeramente más alto

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
            cv::Mat depthImage(height, width, CV_8UC1, cv::Scalar(0)); // Imagen inicial negra
            cv::Mat mask(height, width, CV_8UC1, cv::Scalar(0)); // Máscara inicial negra

            // Factor de escala para convertir profundidad a colores (0-255)
            float scale = 255.0f / (maxDepth - minDepth);

            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    int index = x + y * width;
                    USHORT depth = buffer[index];
                    USHORT realDepth = depth & 0x0fff; // Extraer los 13 bits de profundidad

                    // Si la profundidad está dentro del rango deseado
                    if (realDepth >= minDepth && realDepth <= maxDepth) {
                        BYTE intensity = static_cast<BYTE>((realDepth - minDepth) * scale);
                        depthImage.at<BYTE>(y, x) = intensity;

                        // Crear una máscara que incluya solo el cuerpo
                        mask.at<BYTE>(y, x) = 255; // Incluir este píxel en la máscara
                    }
                }
            }

            // Rellenar huecos en la máscara
            cv::Mat dilatedMask;
            cv::dilate(mask, dilatedMask, cv::Mat(), cv::Point(-1, -1), 2); // Dilatación para interpolar

            // Aplicar suavizado a la imagen de profundidad
            cv::GaussianBlur(depthImage, depthImage, cv::Size(5, 5), 0);

            // Aplicar el mapa de colores
            cv::Mat colorImage;
            cv::applyColorMap(depthImage, colorImage, cv::COLORMAP_JET);

            // Aplicar la máscara dilatada para mostrar el cuerpo completo
            cv::Mat filteredImage;
            colorImage.copyTo(filteredImage, dilatedMask);

            // Mostrar la imagen filtrada
            cv::imshow("Mapa de Elevación Filtrado", filteredImage);

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
