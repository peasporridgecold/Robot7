#include "opencv2/opencv.hpp"
#include <iostream>
#include <ctime>
#include <vector>
#include <time.h>
#include <fstream>
#include <windows.h>
#pragma comment(lib, "winmm.lib")

void playSoundAsync(const char* filename)
{
    PlaySoundA(filename, NULL, SND_FILENAME | SND_ASYNC);
}


int loadHighScore() {
    int high = 0;
    std::ifstream file("high_score.txt");
    if (file.is_open()) {
        file >> high;
        file.close();
    }
    return high;
}

// 최고 기록 저장하기 함수
void saveHighScore(int score) {
    int high = loadHighScore();
    if (score > high) {
        std::ofstream file("high_score.txt");
        if (file.is_open()) {
            file << score;
            file.close();
        }
    }
}

struct Ball
{
    cv::Point position; //x, y
    int radius; //반지름

    Ball()
    {
        this->position = cv::Point();
        this->radius = 0;
    }
};
cv::Point getRandomCirclePosition(int width, int height, int radius)
{
    int x = rand() % (width - 2 * radius) + radius;
    int y = rand() % (height - 2 * radius) + radius;
    return cv::Point(x, y);
}
// ballImg: 넣고 싶은 사진, frame: 웹캠 화면, center: 공의 중심, radius: 반지름
void drawBallImage(cv::Mat& frame, cv::Mat& ballImg, cv::Point center, int radius) {
    if (ballImg.empty()) return;

    // 1. 공 크기에 맞게 이미지 리사이즈 (2*radius x 2*radius)
    cv::Mat resizedBall;
    cv::resize(ballImg, resizedBall, cv::Size(radius * 2, radius * 2));

    // 2. 합성할 영역(ROI) 설정
    int x1 = center.x - radius;
    int y1 = center.y - radius;

    // 화면 밖으로 나가는 경우 예외 처리
    if (x1 < 0 || y1 < 0 || x1 + radius * 2 > frame.cols || y1 + radius * 2 > frame.rows) return;

    cv::Rect roiRect(x1, y1, radius * 2, radius * 2);
    cv::Mat roi = frame(roiRect);

    // 3. 원형 마스크 생성 (검은 바탕에 흰 원)
    cv::Mat mask = cv::Mat::zeros(resizedBall.size(), CV_8UC1);
    cv::circle(mask, cv::Point(radius, radius), radius, cv::Scalar(255), -1);

    // 4. 마스크를 이용하여 합성 (copyTo 사용)
    resizedBall.copyTo(roi, mask);
}
void drawMorphingBall(cv::Mat& frame, const std::vector<cv::Mat>& images, int idx, float a, cv::Point center, int radius) {
    int nextIdx = (idx + 1) % images.size();

    cv::Mat img1, img2, blended;
    cv::resize(images[idx], img1, cv::Size(radius * 2, radius * 2));
    cv::resize(images[nextIdx], img2, cv::Size(radius * 2, radius * 2));

    // 두 이미지를 alpha 비율로 합성 (모핑의 핵심)
    cv::addWeighted(img1, 1.0 - a, img2, a, 0, blended);

    // 이전 답변의 마스크 합성 로직 적용
    cv::Rect roiRect(center.x - radius, center.y - radius, radius * 2, radius * 2);
    if (roiRect.x < 0 || roiRect.y < 0 || roiRect.x + roiRect.width > frame.cols || roiRect.y + roiRect.height > frame.rows) return;

    cv::Mat mask = cv::Mat::zeros(blended.size(), CV_8UC1);
    cv::circle(mask, cv::Point(radius, radius), radius, cv::Scalar(255), -1);

    blended.copyTo(frame(roiRect), mask);
}

void runButterflyGame ()
{
    srand((unsigned int)time(0));
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "웹캠이 없습니다.\n";
        return;
    }

    int width = cvRound(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int height = cvRound(cap.get(cv::CAP_PROP_FRAME_HEIGHT));

    cv::Mat prev_gray;

    Ball ball;
    ball.radius = 20;
    ball.position = getRandomCirclePosition(width, height, ball.radius);

    std::vector<cv::Mat> ballImages = {
        cv::imread("egg.jpg"),
        cv::imread("larva.jpg"),
        cv::imread("pupa.jpg"),
        cv::imread("butterfly1.jpg")
    };

    int totalTime = 40;
    time_t startTime = time(0);
    int score = 0;
    int highScore = loadHighScore();

    int currentStage = 0;
    float alpha = 0.0f;
    bool isMorphing = false;
    int finalRadius = 60;

    bool gameOver = false;
    bool isFever = false;
    bool feverUsed = false;

    std::vector<Ball> feverBalls;

    while (true)
    {
        cv::Mat frame, gray, diff, thresh;
        cap >> frame;
        if (frame.empty()) break;

        cv::flip(frame, frame, 1);

        int remainingTime = totalTime - (int)(time(0) - startTime);
        if (remainingTime <= 0) {
            remainingTime = 0;
            gameOver = true;
        }

        if (!gameOver)
        {
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            cv::GaussianBlur(gray, gray, cv::Size(15, 15), 0);

            if (!prev_gray.empty()) {
                cv::absdiff(prev_gray, gray, diff);
                cv::threshold(diff, thresh, 25, 255, cv::THRESH_BINARY);
            }
            gray.copyTo(prev_gray);
        }

        /* ================== FEVER START ================== */
        if (!gameOver && remainingTime <= 10 && !feverUsed)
        {
            isFever = true;
            feverUsed = true;

            playSoundAsync("fevertime.wav");

            feverBalls.clear();

            for (int i = 0; i < 5; i++) {
                Ball b;
                b.radius = 40;
                b.position = getRandomCirclePosition(width, height, b.radius);
                feverBalls.push_back(b);
            }
        }

        /* ================== GAME OVER ================== */
        if (gameOver)
        {
            thresh.release();

            static bool gameOverSoundPlayed = false;
            if (gameOver && !gameOverSoundPlayed)
            {
                playSoundAsync("gameover.wav");
                gameOverSoundPlayed = true;
            }

            static bool saved = false;
            if (!saved) {
                saveHighScore(score);
                highScore = loadHighScore();
                saved = true;
            }

            cv::putText(frame, "GAME OVER", { width / 2 - 150, height / 2 },
                cv::FONT_HERSHEY_DUPLEX, 2, { 0,0,255 }, 3);
            cv::putText(frame, "Final Score: " + std::to_string(score),
                { width / 2 - 150, height / 2 + 60 },
                cv::FONT_HERSHEY_PLAIN, 2, { 255,255,255 }, 2);
            cv::putText(frame, "Best: " + std::to_string(highScore),
                { width / 2 - 150, height / 2 + 100 },
                cv::FONT_HERSHEY_PLAIN, 2, { 0,255,255 }, 2);
        }

        /* ================== FEVER MODE ================== */
        else if (isFever)
        {
            cv::applyColorMap(frame, frame, cv::COLORMAP_JET);

            for (auto& b : feverBalls)
            {
                int x1 = max(0, b.position.x - b.radius);
                int y1 = max(0, b.position.y - b.radius);
                int x2 = min(width, b.position.x + b.radius);
                int y2 = min(height, b.position.y + b.radius);

                cv::Rect r(x1, y1, x2 - x1, y2 - y1);

                if (!thresh.empty() && r.area() > 0)
                {
                    if (cv::countNonZero(thresh(r)) > r.area() * 0.1)
                    {
                        score += 2;
                        b.position = getRandomCirclePosition(width, height, b.radius);
                    }
                }
                drawBallImage(frame, ballImages.back(), b.position, b.radius);
            }
            cv::putText(frame, "FEVER TIME!",
                { width / 2 - 150, 80 },
                cv::FONT_HERSHEY_DUPLEX, 2, { 0,255,255 }, 3);
        }

        /* ================== NORMAL MODE ================== */
        else
        {
            int x1 = max(0, ball.position.x - ball.radius);
            int y1 = max(0, ball.position.y - ball.radius);
            int x2 = min(width, ball.position.x + ball.radius);
            int y2 = min(height, ball.position.y + ball.radius);

            cv::Rect r(x1, y1, x2 - x1, y2 - y1);

            if (!thresh.empty() && r.area() > 0)
            {
                if (cv::countNonZero(thresh(r)) > r.area() * 0.1 && !isMorphing)
                {
                    score++;
                    playSoundAsync("hit.wav");
                    ball.position = getRandomCirclePosition(width, height, ball.radius);

                    if (score % 10 == 0 && currentStage < 3) {
                        isMorphing = true;
                        alpha = 0.0f;
                        playSoundAsync("evolution.wav");
                    }
                }
            }

            if (isMorphing && currentStage < 3)
            {
                alpha += 0.05f;

                if (currentStage == 2)
                    ball.radius = 20 + (int)((finalRadius - 20) * alpha);

                if (alpha >= 1.0f)
                {
                    alpha = 0;
                    isMorphing = false;
                    currentStage++;

                    if (currentStage == 3)
                        ball.radius = finalRadius;
                }

                drawMorphingBall(frame, ballImages, currentStage, alpha,
                    ball.position, ball.radius);
            }
            else
            {
                drawBallImage(frame, ballImages[currentStage],
                    ball.position, ball.radius);
            }

        }

        /* ================== UI ================== */
        cv::putText(frame, "Score: " + std::to_string(score),
            { 20,30 }, cv::FONT_HERSHEY_PLAIN, 2, { 255,255,255 }, 2);
        cv::putText(frame, "Time: " + std::to_string(remainingTime),
            { width - 200,30 }, cv::FONT_HERSHEY_PLAIN, 2,
            remainingTime <= 5 ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 255, 255), 2);

        cv::imshow("GAME", frame);
        if (cv::waitKey(10) == 27) break;
    }

    cap.release();
    cv::destroyAllWindows();
}
