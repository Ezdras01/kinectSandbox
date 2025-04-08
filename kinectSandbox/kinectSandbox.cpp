#include <Windows.h>
#include <NuiApi.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

// Asigna colores según profundidad (gradiente Rojo → Amarillo → Verde)
cv::Vec3b procesarProfundidad(USHORT realDepth, USHORT min, USHORT max, const cv::Mat& waterEffect, int x, int y) {
    if (realDepth >= min && realDepth <= max) {
        float position = static_cast<float>(realDepth - min) / (max - min);

        if (position < 0.5f) {
            float t = position * 2.0f;
            return cv::Vec3b(0, static_cast<uchar>(t * 255), 255); // Rojo a amarillo
        }
        else {
            float t = (position - 0.5f) * 2.0f;
            return cv::Vec3b(0, 255, static_cast<uchar>((1.0f - t) * 255)); // Amarillo a verde
        }
    }
    return waterEffect.at<cv::Vec3b>(y, x); // Si está fuera de rango → agua
}

int main() {
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

    HANDLE depthStream = nullptr;
    hr = sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, NUI_IMAGE_RESOLUTION_640x480, 0, 2, nullptr, &depthStream);

    const int width = 640, height = 480;
    const USHORT minDepth = 800, maxDepth = 1500;
    cv::Mat waterEffect;
    float rippleTime = 0.0f;
    int frameCounter = 0;
    const int waterUpdateRate = 5;

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

            // Actualizar efecto agua
            frameCounter++;
            if (waterEffect.empty() || frameCounter % waterUpdateRate == 0) {
                waterEffect = cv::Mat(height, width, CV_8UC3);
                cv::Scalar baseColor(220, 200, 150);
                cv::Scalar variance(30, 30, 30);
                cv::randu(waterEffect, baseColor - variance, baseColor + variance);

                rippleTime += 0.15f;
                for (int y = 0; y < height; y++) {
                    for (int x = 0; x < width; x++) {
                        float wave1 = 8.0f * sin(y * 0.03f + rippleTime * 1.3f);
                        float wave2 = 5.0f * cos(x * 0.02f + rippleTime * 0.7f);
                        float wave3 = 3.0f * sin((x + y) * 0.01f + rippleTime * 0.5f);

                        int offsetX = static_cast<int>(wave1 + wave3);
                        int offsetY = static_cast<int>(wave2 + wave3);
                        int newX = cv::borderInterpolate(x + offsetX, width, cv::BORDER_REFLECT);
                        int newY = cv::borderInterpolate(y + offsetY, height, cv::BORDER_REFLECT);

                        waterEffect.at<cv::Vec3b>(y, x) = waterEffect.at<cv::Vec3b>(newY, newX);

                        // Espuma (brillo aleatorio)
                        if ((x + y + static_cast<int>(rippleTime * 10)) % 30 == 0) {
                            for (int c = 0; c < 3; ++c)
                                waterEffect.at<cv::Vec3b>(y, x)[c] = std::min(255, waterEffect.at<cv::Vec3b>(y, x)[c] + 50);
                        }
                    }
                }
            }

            // Colorear imagen según profundidad
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    int index = x + y * width;
                    USHORT realDepth = buffer[index] & 0x0fff;
                    colorImage.at<cv::Vec3b>(y, x) = procesarProfundidad(realDepth, minDepth, maxDepth, waterEffect, x, y);
                }
            }

            // Mostrar info
            cv::putText(colorImage, "Rango Valido: " + std::to_string(minDepth / 1000.0f) + "m - " + std::to_string(maxDepth / 1000.0f) + "m",
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
            cv::putText(colorImage, "Fuera de rango: Efecto agua", cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

            cv::imshow("Mapa de Profundidad con Efecto Agua", colorImage);
        }

        texture->UnlockRect(0);
        sensor->NuiImageStreamReleaseFrame(depthStream, &imageFrame);

        if (cv::waitKey(30) == 27) break;
    }

    sensor->NuiShutdown();
    sensor->Release();
    return 0;
}
