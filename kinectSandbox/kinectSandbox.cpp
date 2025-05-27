#define _CRT_SECURE_NO_WARNINGS
#include <Windows.h>
#include <NuiApi.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm> // Asegúrate de incluir esta cabecera para std::
#include <wchar.h>   // Para funciones de conversión de cadenas
#include <errno.h>

//ESTO ESTA EN LA NUBE

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
    USHORT minDepth = 500;   // Profundidad mínima en milímetros (0.8 metros)
    USHORT maxDepth = 4000; // Profundidad máxima en milímetros (4 metros)
    USHORT depthStep = 50;

    bool fullScreen = false; // Variable para controlar el estado de pantalla completa
    HWND windowHandle = nullptr; // Handle de la ventana

    bool savingData = false;
    std::ofstream outputFile;
    std::string filename = "depth_matrix.csv";

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
            float scale = 255.0f / (minDepth - maxDepth);

            float depthRange = static_cast<float>(maxDepth - minDepth);
            std::vector<std::vector<USHORT>> depthMatrixToSave(height, std::vector<USHORT>(width));

            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    int index = x + y * width;
                    USHORT depth = buffer[index];
                    USHORT realDepth = depth & 0x0fff; // Extraer los 13 bits de profundidad

                    depthMatrixToSave[y][x] = realDepth;

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
            cv::flip(colorImage, colorImage, 1); // Corrige imagen invertida


            // Obtener tamaño de la pantalla
            int screenW = GetSystemMetrics(SM_CXSCREEN);
            int screenH = GetSystemMetrics(SM_CYSCREEN);

            // Redimensionar imagen a pantalla completa
            cv::Mat escalada;
            cv::resize(colorImage, escalada, cv::Size(screenW, screenH));


            wchar_t wTitle[27];
            mbstowcs(wTitle, "Mapa de Elevación en Color", 27);

            if (!windowHandle) {
                windowHandle = FindWindow(nullptr, wTitle);
            }

            if (savingData && outputFile.is_open()) {
                for (int y = 0; y < height; ++y) {
                    for (int x = 0; x < width; ++x) {
                        outputFile << depthMatrixToSave[y][x];
                        if (x < width - 1) {
                            outputFile << ",";
                        }
                    }
                    outputFile << "\n";
                }
                std::cout << "Matriz de profundidad guardada en " << filename << std::endl;
                savingData = false; // Desactivar la bandera después de guardar
                outputFile.close();
            }

            // Crear las cadenas de texto para mostrar la profundidad
            std::string minDepthText = "Min: " + std::to_string(minDepth) + "mm";
            std::string maxDepthText = "Max: " + std::to_string(maxDepth) + "mm";

            // Especificar la posición del texto
            cv::Point minDepthPos(10, 30); // Esquina superior izquierda
            cv::Point maxDepthPos(10, 60); // Debajo del texto de profundidad mínima

            // Especificar la fuente y otros parámetros del texto
            int fontFace = cv::FONT_HERSHEY_SIMPLEX;
            double fontScale = 1.0;
            cv::Scalar textColor(255, 255, 255); // Color blanco
            int thickness = 2;

            // Dibujar el texto en la imagen a color
            cv::putText(colorImage, minDepthText, minDepthPos, fontFace, fontScale, textColor, thickness);
            cv::putText(colorImage, maxDepthText, maxDepthPos, fontFace, fontScale, textColor, thickness);


            // Mostrar el mapa de elevación en color
            cv::imshow("Mapa de Elevación en Color", escalada);

            // Salir si se presiona 'Esc'
            /*if (cv::waitKey(1) == 27) {
                break;
            }*/

            int key = cv::waitKey(1);

            if (key == 'f' && windowHandle) {
                fullScreen = !fullScreen;
                if (fullScreen) {
                    SetWindowLongPtr(windowHandle, GWL_STYLE, WS_POPUP | WS_VISIBLE);
                    SetWindowPos(windowHandle, HWND_TOP, 0, 0, GetSystemMetrics(SM_CXSCREEN), GetSystemMetrics(SM_CYSCREEN), SWP_SHOWWINDOW);
                }
                else {
                    SetWindowLongPtr(windowHandle, GWL_STYLE, WS_OVERLAPPEDWINDOW | WS_VISIBLE);
                    SetWindowPos(windowHandle, nullptr, 100, 100, width + 16, height + 39, SWP_SHOWWINDOW); // Usar nullptr en lugar de HWND_NORMAL
                }
            }

            // Ajustar la profundidad mínima y máxima según la tecla presionada
            if (key == 'a') {
                minDepth = std::max(static_cast<USHORT>(0), static_cast<USHORT>(minDepth - depthStep));
                std::cout << "Profundidad mínima: " << minDepth << " mm" << std::endl;
            }
            else if (key == 's') {
                minDepth += depthStep;
                std::cout << "Profundidad mínima: " << minDepth << " mm" << std::endl;
            }
            else if (key == 'z') {
                maxDepth = std::max<USHORT>(minDepth + 1, static_cast<USHORT>(maxDepth - depthStep));
                std::cout << "Profundidad máxima: " << maxDepth << " mm" << std::endl;
            }
            else if (key == 'x') {
                maxDepth += depthStep;
                std::cout << "Profundidad máxima: " << maxDepth << " mm" << std::endl;
            }
            else if (key == 'g') {
                // Activar la bandera para guardar los datos en el próximo frame
                savingData = true;
                outputFile.open(filename, std::ios::out);
                if (!outputFile.is_open()) {
                    std::cerr << "No se pudo abrir el archivo " << filename << " para escritura." << std::endl;
                    savingData = false; // Asegurarse de que la bandera se desactive si falla la apertura
                }
            }
            else if (key == 27) { // Salir si se presiona 'Esc'
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
