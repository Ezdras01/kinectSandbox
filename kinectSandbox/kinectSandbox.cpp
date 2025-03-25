#include <Windows.h>
#include <NuiApi.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

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
    const USHORT minDepth = 800;   // 0.8 metros
    const USHORT maxDepth = 1500;  // 1.5 metros

    // Variables para el efecto de agua
    cv::Mat waterEffect;
    float rippleTime = 0.0f;
    const int waterUpdateRate = 5;  // Actualizar efecto de agua cada X frames
    int frameCounter = 0;

    while (true) {
        NUI_IMAGE_FRAME imageFrame;
        hr = sensor->NuiImageStreamGetNextFrame(depthStream, 0, &imageFrame);
        if (FAILED(hr)) continue;

        INuiFrameTexture* texture = imageFrame.pFrameTexture;
        NUI_LOCKED_RECT lockedRect;
        texture->LockRect(0, &lockedRect, nullptr, 0);

        if (lockedRect.Pitch != 0) {
            USHORT* buffer = (USHORT*)lockedRect.pBits;
            cv::Mat colorImage(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

            // Crear/actualizar efecto de agua
            frameCounter++;
            if (waterEffect.empty() || frameCounter % waterUpdateRate == 0) {
                waterEffect = cv::Mat(height, width, CV_8UC3);

                cv::Scalar waterBaseColor(220, 200, 150);  // Azul claro (B=220, G=200, R=150)
                cv::Scalar waterVariance(30, 30, 30);     // Variación permitida en el color

                // Base azul con variaciones sutiles
                cv::randu(waterEffect, waterBaseColor - waterVariance,
                    waterBaseColor + waterVariance);

                // Añadir patrones de olas más definidos
                for (int y = 0; y < height; y++) {
                    for (int x = 0; x < width; x++) {
                        if (y % 30 == 0 || x % 40 == 0) {
                            waterEffect.at<cv::Vec3b>(y, x) += cv::Vec3b(10, 5, -5);
                        }

                        float depthFactor = 1.0f - (y / float(height) * 0.5f);
                        waterEffect.at<cv::Vec3b>(y, x) = cv::Vec3b(
                            waterEffect.at<cv::Vec3b>(y, x)[0] * depthFactor,
                            waterEffect.at<cv::Vec3b>(y, x)[1] * depthFactor,
                            waterEffect.at<cv::Vec3b>(y, x)[2] * depthFactor
                        );
                    }
                }
            }

            // Aplicar distorsión de onda
            rippleTime += 0.15f;
            cv::Mat waterDistorted;
            waterEffect.copyTo(waterDistorted);

            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    // Distorsión en forma de onda
                    float wave1 = 8.0f * sin(y * 0.03f + rippleTime * 1.3f);
                    float wave2 = 5.0f * cos(x * 0.02f + rippleTime * 0.7f);
                    float wave3 = 3.0f * sin((x + y) * 0.01f + rippleTime * 0.5f);

                    int offsetX = static_cast<int>(wave1 + wave3);
                    int offsetY = static_cast<int>(wave2 + wave3);

                    int newX = cv::borderInterpolate(x + offsetX, width, cv::BORDER_REFLECT);
                    int newY = cv::borderInterpolate(y + offsetY, height, cv::BORDER_REFLECT);

                    waterDistorted.at<cv::Vec3b>(y, x) = waterEffect.at<cv::Vec3b>(newY, newX);

                    // Añadir brillo para efecto de espuma
                    if ((x + y + static_cast<int>(rippleTime * 10)) % 30 == 0) {
                        waterDistorted.at<cv::Vec3b>(y, x) = cv::Vec3b(
                            std::min(255, waterDistorted.at<cv::Vec3b>(y, x)[0] + 50),  // Aumentar azul
                            std::min(255, waterDistorted.at<cv::Vec3b>(y, x)[1] + 50),  // Aumentar verde
                            std::min(255, waterDistorted.at<cv::Vec3b>(y, x)[2] + 50)   // Aumentar rojo
                        );
                    }
                }
            }

            // Procesar datos de profundidad
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    int index = x + y * width;
                    USHORT realDepth = buffer[index] & 0x0fff;

                    if (realDepth >= minDepth && realDepth <= maxDepth) {
                        float position = static_cast<float>(realDepth - minDepth) / (maxDepth - minDepth);

                        if (position < 0.5f) {
                            // Rojo -> Amarillo
                            float t = position * 2.0f;
                            colorImage.at<cv::Vec3b>(y, x) = cv::Vec3b(
                                0,                                      // B
                                static_cast<uchar>(t * 255),              // G
                                255                                      // R
                            );
                        }
                        else {
                            // Amarillo -> Verde
                            float t = (position - 0.5f) * 2.0f;
                            colorImage.at<cv::Vec3b>(y, x) = cv::Vec3b(
                                0,                                      // B
                                255,                                    // G
                                static_cast<uchar>((1.0f - t) * 255)    // R
                            );
                        }
                    }
                    else {
                        // Usar efecto de agua para áreas fuera de rango
                        colorImage.at<cv::Vec3b>(y, x) = waterDistorted.at<cv::Vec3b>(y, x);
                    }
                }
            }

            // Mostrar información
            std::string depthText = "Rango Valido: " +
                std::to_string(minDepth / 1000.0f) + "m - " +
                std::to_string(maxDepth / 1000.0f) + "m";

            cv::putText(colorImage, depthText, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

            cv::putText(colorImage, "Fuera de rango: Efecto agua", cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

            cv::imshow("Mapa de Profundidad con Efecto Agua", colorImage);
        }

        texture->UnlockRect(0);
        sensor->NuiImageStreamReleaseFrame(depthStream, &imageFrame);

        if (cv::waitKey(30) == 27) break; // Mayor tiempo para mejor animación
    }

    sensor->NuiShutdown();
    sensor->Release();
    return 0;
}