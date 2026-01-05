#include <opencv2/opencv.hpp>
#include <iostream>
#include <ctime>
#include <vector>
#include <fstream>
#include <windows.h>
#pragma comment(lib, "winmm.lib")

using namespace cv;
using namespace std;
extern void runButterflyGame();

/* ===================== 공용 유틸 ===================== */
Point getRandomRectPosition(int w, int h, int size)
{
    return Point(rand() % (w - size), rand() % (h - size));
}


/* =====================================================
   ===============얼굴 게임 =========================
   ===================================================== */

struct RectBall {
    Point pos;
    int size;
    Mat image;
    string currentEffect;
};

vector<string> transformNames = {
    "Flip Horizontal",
    "Flip Vertical",
    "Blur",
    "Morphological Closing",
    "Zoom In",
    "Color Invert"
};


void transformBall(Mat& img, string& effectName)
{
    int mode = rand() % transformNames.size();
    effectName = transformNames[mode];

    switch (mode)
    {
    case 0: flip(img, img, 1); break; //좌우대칭
    case 1: flip(img, img, 0); break; //상하대칭
    case 2: cv::GaussianBlur(img, img, Size(1, 1), 0); break;
    case 3:
        morphologyEx(img, img, MORPH_CLOSE,
            getStructuringElement(MORPH_RECT, Size(3, 3)));
        break;
    case 4:
    {
        double scale = 1.4;
        Mat enlarged;
        resize(img, enlarged, Size(), scale, scale);

        int x = (enlarged.cols - img.cols) / 2;
        int y = (enlarged.rows - img.rows) / 2;

        img = enlarged(Rect(x, y, img.cols, img.rows)).clone();
        break;
    }
    case 5:
    {
        // 색 반전 (Negative)
        cv::bitwise_not(img, img);
        break;
    }
    }
}

void runFaceGame()
{
    VideoCapture cap(0);
    if (!cap.isOpened()) return;

    const wchar_t* touchSound = L"touch.wav";

    int w = cap.get(CAP_PROP_FRAME_WIDTH);
    int h = cap.get(CAP_PROP_FRAME_HEIGHT);

    Rect guide(w / 2 - 100, h / 2 - 100, 200, 200);
    RectBall ball;
    ball.size = 80;
    ball.pos = getRandomRectPosition(w, h, ball.size);

    Mat captured, prevGray;

    // 1단계: 얼굴 캡처 루프
    while (true)
    {
        Mat frame;
        cap >> frame;
        if (frame.empty()) break;
        flip(frame, frame, 1);

        rectangle(frame, guide, { 0,255,0 }, 3);
        putText(frame, "Press SPACE to Capture", { guide.x - 20, guide.y - 10 },
            FONT_HERSHEY_PLAIN, 1.5, { 0,255,0 }, 2);

        cv::imshow("FACE GAME", frame);

        int key = waitKey(10);
        if (key == ' ') { // 스페이스바를 누르면 캡처
            captured = frame(guide).clone();
            cv::resize(captured, ball.image, Size(80, 80)); //크기 미리 조절
            break;
        }
        else if (key == 27) return;
    }
    destroyAllWindows();//창 닫기

    // 2단계: 실제 게임 루프
    while (true)
    {
        Mat frame, gray, diff, thresh;
        cap >> frame;
        if (frame.empty()) break;
        flip(frame, frame, 1);

        cvtColor(frame, gray, COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray, Size(15, 15), 0);

        if (!prevGray.empty())
        {
            absdiff(prevGray, gray, diff);
            threshold(diff, thresh, 25, 255, THRESH_BINARY);

            Rect r(ball.pos.x, ball.pos.y, ball.size, ball.size);
            // 경계값 검사 (영역이 프레임을 벗어나지 않게)
            r &= Rect(0, 0, w, h);

            if (!thresh.empty() && countNonZero(thresh(r)) > r.area() * 0.1)
            {
                PlaySound(touchSound, NULL, SND_FILENAME | SND_ASYNC | SND_NODEFAULT);

                transformBall(ball.image, ball.currentEffect);
                ball.pos = getRandomRectPosition(w, h, ball.size);
            }
        }
        gray.copyTo(prevGray);

        // 출력할 때만 리사이즈해서 복사 (원본 보존)
        Rect r(ball.pos.x, ball.pos.y, ball.size, ball.size);
        r &= Rect(0, 0, w, h);

        Mat temp;
        cv::resize(ball.image, temp, Size(r.width, r.height));
        temp.copyTo(frame(r));

        putText(frame,
            "Effect: " + ball.currentEffect,
            Point(20, frame.rows - 20),
            FONT_HERSHEY_PLAIN,
            2,
            Scalar(0, 255, 255),
            2
        );
        putText(frame,
            "Press ESC to Exit",
            Point(20, 40),
            FONT_HERSHEY_PLAIN,
            2,
            Scalar(0, 255, 0),
            2
        );


        cv::imshow("FACE GAME", frame);
        if (waitKey(10) == 27) break;
    }
}

/* =====================================================
   ===============모드 선택 =========================
   ===================================================== */

int selectMode()
{
    VideoCapture cap(0);
    Mat prevGray;

    Rect left(0, 0, 320, 480);
    Rect right(320, 0, 320, 480);

    while (true)
    {
        Mat frame, gray, diff, thresh;
        cap >> frame;
        if (frame.empty()) break;
        flip(frame, frame, 1);

        rectangle(frame, left, { 255,0,0 }, 3);
        rectangle(frame, right, { 0,255,0 }, 3);

        putText(frame, "1. BUTTERFLY GAME",
            { 20,240 }, FONT_HERSHEY_DUPLEX, 0.9, { 255,255,255 }, 2);
        putText(frame, "(Competition)",
            { 40,300 }, FONT_HERSHEY_DUPLEX, 1, { 255,255,255 }, 2);
        putText(frame, "2. FACE GAME",
            { 380,240 }, FONT_HERSHEY_DUPLEX, 0.9, { 255,255,255 }, 2);
        putText(frame, "(Solo)",
            { 400,300 }, FONT_HERSHEY_DUPLEX, 1, { 255,255,255 }, 2);


        cvtColor(frame, gray, COLOR_BGR2GRAY);
        GaussianBlur(gray, gray, Size(15, 15), 0);

        if (!prevGray.empty())
        {
            absdiff(prevGray, gray, diff);
            threshold(diff, thresh, 25, 255, THRESH_BINARY);

            if (countNonZero(thresh(left)) > left.area() * 0.1) {
                cap.release();
                destroyAllWindows();
                return 1;
            }
            if (countNonZero(thresh(right)) > right.area() * 0.1) {
                cap.release();
                destroyAllWindows();
                return 2;
            }
        }
        gray.copyTo(prevGray);

        imshow("SELECT MODE", frame);
        if (waitKey(10) == 27) break;
    }
    return 0;
}

/* ===================== main ===================== */
int main()
{
    srand((unsigned)time(0));
    int mode = selectMode();

    if (mode == 1) runButterflyGame();
    else if (mode == 2) runFaceGame();

    return 0;
}
