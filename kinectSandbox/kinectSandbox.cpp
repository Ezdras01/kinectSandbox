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
        0, 2, nullptr, &depthStream
    );

    const int width = 640;
    const int height = 480;

    // Rango de profundidad permitido (ajústalo según tu sandbox)
    const USHORT minDepth = 800;   // Profundidad mínima en mm (0.8 metros)
    const USHORT maxDepth = 1500;  // Profundidad máxima en mm (1.5 metros)

    while (true) {
        NUI_IMAGE_FRAME imageFrame;
        hr = sensor->NuiImageStreamGetNextFrame(depthStream, 0, &imageFrame);
        if (FAILED(hr)) continue;

        INuiFrameTexture* texture = imageFrame.pFrameTexture;
        NUI_LOCKED_RECT lockedRect;
        texture->LockRect(0, &lockedRect, nullptr, 0);

        if (lockedRect.Pitch != 0) {
            USHORT* buffer = (USHORT*)lockedRect.pBits;
            cv::Mat depthImage(height, width, CV_8UC1, cv::Scalar(0));
            cv::Mat colorImage;

            // Normalización de los datos de profundidad al rango 0-255
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    int index = x + y * width;
                    USHORT realDepth = buffer[index] & 0x0fff;

                    // Normalizar solo los valores dentro del rango permitido
                    if (realDepth >= minDepth && realDepth <= maxDepth) {
                        BYTE intensity = static_cast<BYTE>(((realDepth - minDepth) * 255.0f) / (maxDepth - minDepth));
                        depthImage.at<BYTE>(y, x) = intensity;
                    }
                    else {
                        depthImage.at<BYTE>(y, x) = 0;
                    }
                }
            }

            // Aplicar mapa de colores continuo
            cv::applyColorMap(depthImage, colorImage, cv::COLORMAP_JET);

            // Mostrar la imagen normalizada con mapa de elevación continuo
            cv::imshow("Mapa de Elevación Continuo", colorImage);

            // Salir si se presiona 'Esc'
            if (cv::waitKey(1) == 27) break;
        }

        texture->UnlockRect(0);
        sensor->NuiImageStreamReleaseFrame(depthStream, &imageFrame);
    }

    sensor->NuiShutdown();
    sensor->Release();
    return 0;
}
