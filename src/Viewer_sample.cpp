#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include <iomanip>

using namespace std;

// 带进度条的步骤输出
void printStep(const string& step, int seconds) {
    cout << "     " <<step << " ..." << endl;

    const int barWidth = 80; // 进度条宽度
    for (int i = 0; i <= barWidth; ++i) {
        float progress = (float)i / barWidth;
        int elapsed = (int)(progress * seconds * 1000); // 已经消耗的时间（毫秒）

        // 打印进度条
        cout << "   " << "[";
        int pos = i;
        for (int j = 0; j < barWidth; ++j) {
            if (j < pos) cout << "=";
            else if (j == pos) cout << ">";
            else cout << " ";
        }
        cout << "     " << "] " << setw(3) << (int)(progress * 100) << "%\r";
        cout.flush();

        // 控制进度条刷新速度
        this_thread::sleep_for(chrono::milliseconds(seconds * 1000 / barWidth));
    }
    cout << endl;

    cout << "     " << "[完成] " << step << endl << endl;
}

int main() {
    std::this_thread::sleep_for(15s);
    cout << "   ==============================" << endl;
    cout << "       启动数据处理与分析流程" << endl;
    cout << "   ==============================" << endl << endl;

    printStep("加载多源数据", 3);
    printStep("数据清理与异常值去除", 5);
    printStep("数据统一化处理", 4);
    printStep("构建数据索引加速结构", 3);
    printStep("基础特征计算", 8);
    printStep("高级特征提取", 10);
    printStep("特征降维编码", 10);
    printStep("多模态特征融合", 5);
    printStep("基于空间语义的提取", 14);
    printStep("多尺度结构分析", 12);
    printStep("模式识别建模", 17);
    printStep("特征向量化编码SV ", 17);
    printStep("目标识别输出", 12);


    cout << "   ==============================" << endl;
    cout << "           处理完成 " << endl;
    cout << "   ==============================" << endl;

    std::cin.get();
    return 0;
}
