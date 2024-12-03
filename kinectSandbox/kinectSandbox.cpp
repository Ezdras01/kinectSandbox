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

    // Inicializar el flujo de profundidad
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

    // Definir el rango de profundidad para el mapeo
    const USHORT minDepth = 500;   // Profundidad mínima en milímetros (0.8 metros)
    const USHORT maxDepth = 4000; // Profundidad máxima en milímetros (4 metros)

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
            cv::Mat depthImage(height, width, CV_8UC1);

            // Calcular el factor de escala para mapear la profundidad al rango 0-255
            float scale = 255.0f / (maxDepth - minDepth);

            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    int index = x + y * width;
                    USHORT depth = buffer[index];
                    USHORT realDepth = depth & 0x0fff; // Extraer los 13 bits de profundidad

                    BYTE intensity = 0;

                    // Verificar si la profundidad está dentro del rango deseado
                    if (realDepth >= minDepth && realDepth <= maxDepth) {
                        // Mapear la profundidad al rango de intensidad 0-255
                        intensity = static_cast<BYTE>((realDepth - minDepth) * scale);
                    }
                    else {
                        // Fuera del rango, asignar negro (intensidad mínima)
                        intensity = 0;
                    }

                    depthImage.at<BYTE>(y, x) = intensity;
                }
            }

            // Aplicar el mapa de colores
            cv::Mat colorImage;
            cv::applyColorMap(depthImage, colorImage, cv::COLORMAP_JET);

            // Mostrar el mapa de elevación en color
            cv::imshow("Mapa de Elevación en Color", colorImage);

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
