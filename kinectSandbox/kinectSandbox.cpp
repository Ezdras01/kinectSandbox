#include <Windows.h>
#include <NuiApi.h>
#include <opencv2/opencv.hpp>
#include <iostream>

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
    hr = sensor->NuiImageStreamOpen(
        NUI_IMAGE_TYPE_DEPTH,
        NUI_IMAGE_RESOLUTION_640x480,
        0, 2, nullptr, &depthStream
    );

    const int width = 640;
    const int height = 480;

    const USHORT minDepth = 800;   // Profundidad mínima ajustada
    const USHORT maxDepth = 1500; // Profundidad máxima ajustada
    const USHORT interval = 50;   // Intervalo para contornos

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

            float scale = 255.0f / (maxDepth - minDepth);

            // Convertir profundidad a escala de grises
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    int index = x + y * width;
                    USHORT realDepth = buffer[index] & 0x0fff;

                    if (realDepth >= minDepth && realDepth <= maxDepth) {
                        depthImage.at<BYTE>(y, x) = static_cast<BYTE>((realDepth - minDepth) * scale);
                    }
                    else {
                        depthImage.at<BYTE>(y, x) = 0;
                    }
                }
            }

            // Suavizar la imagen para reducir ruido
            cv::GaussianBlur(depthImage, depthImage, cv::Size(5, 5), 0);

            // Aplicar mapa de colores
            cv::applyColorMap(depthImage, colorImage, cv::COLORMAP_JET);

            // Dibujar contornos topográficos
            for (int i = 0; i <= 255; i += interval) {
                cv::Mat mask = (depthImage >= i) & (depthImage < i + interval);
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                cv::drawContours(colorImage, contours, -1, cv::Scalar(0, 0, 0), 1);
            }

            cv::imshow("Mapa Topográfico con Contornos", colorImage);

            if (cv::waitKey(1) == 27) break;
        }

        texture->UnlockRect(0);
        sensor->NuiImageStreamReleaseFrame(depthStream, &imageFrame);
    }

    sensor->NuiShutdown();
    sensor->Release();
    return 0;
}
