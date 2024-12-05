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

    // Niveles de elevación en mm (modifica según tu proyecto)
    const USHORT nivel1 = 900;   // Nivel bajo (agua, azul)
    const USHORT nivel2 = 1100;  // Nivel medio (tierra, verde)
    const USHORT nivel3 = 1300;  // Nivel alto (montañas, marrón)

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
            cv::Mat depthImage(height, width, CV_8UC3, cv::Scalar(0, 0, 0)); // Imagen en negro con 3 canales (BGR)

            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    int index = x + y * width;
                    USHORT depth = buffer[index];
                    USHORT realDepth = depth & 0x0fff; // Extraer los 13 bits de profundidad

                    cv::Vec3b color(0, 0, 0); // Color predeterminado (negro)

                    // Clasificar en niveles de elevación
                    if (realDepth >= minDepth && realDepth < nivel1) {
                        // Nivel 1 (agua, azul)
                        color = cv::Vec3b(255, 0, 0); // Azul
                    }
                    else if (realDepth >= nivel1 && realDepth < nivel2) {
                        // Nivel 2 (tierra, verde)
                        color = cv::Vec3b(0, 255, 0); // Verde
                    }
                    else if (realDepth >= nivel2 && realDepth < nivel3) {
                        // Nivel 3 (montañas, marrón)
                        color = cv::Vec3b(42, 42, 165); // Marrón
                    }
                    else if (realDepth >= nivel3 && realDepth <= maxDepth) {
                        // Nivel 4 (alturas extremas, blanco)
                        color = cv::Vec3b(255, 255, 255); // Blanco
                    }

                    // Asignar el color al píxel
                    depthImage.at<cv::Vec3b>(y, x) = color;
                }
            }

            // Mostrar la imagen con niveles de elevación
            cv::imshow("Mapa de Elevación con Niveles", depthImage);

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
