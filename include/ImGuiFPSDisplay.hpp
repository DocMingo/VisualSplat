#ifdef USE_IMGUI
#include <imgui/imgui.h>
#include <imgui/backends/imgui_impl_glfw.h>
#include <imgui/backends/imgui_impl_opengl3.h>

#include <chrono>
#include <deque>
#include <numeric>
#include <iomanip>
#include <sstream>

// 1. 简单的FPS计数器类
class FPSCounter {
private:
    std::chrono::high_resolution_clock::time_point lastTime;
    std::deque<double> frameTimes;
    double totalTime;
    int frameCount;
    double currentFPS;
    double averageFPS;
    double minFPS;
    double maxFPS;

    static constexpr int SAMPLE_COUNT = 60; // 保存60帧的数据用于平均计算

public:
    FPSCounter() :
        totalTime(0.0),
        frameCount(0),
        currentFPS(0.0),
        averageFPS(0.0),
        minFPS(999.0),
        maxFPS(0.0) {
        lastTime = std::chrono::high_resolution_clock::now();
    }

    void update() {
        auto currentTime = std::chrono::high_resolution_clock::now();
        double deltaTime = std::chrono::duration<double>(currentTime - lastTime).count();
        lastTime = currentTime;

        if (deltaTime > 0.0) {
            currentFPS = 1.0 / deltaTime;

            // 更新最小最大值
            minFPS = std::min(minFPS, currentFPS);
            maxFPS = std::max(maxFPS, currentFPS);

            // 保存帧时间用于平滑计算
            frameTimes.push_back(deltaTime);
            totalTime += deltaTime;

            if (frameTimes.size() > SAMPLE_COUNT) {
                totalTime -= frameTimes.front();
                frameTimes.pop_front();
            }

            // 计算平均FPS
            if (!frameTimes.empty()) {
                averageFPS = frameTimes.size() / totalTime;
            }

            frameCount++;
        }
    }

    double getCurrentFPS() const { return currentFPS; }
    double getAverageFPS() const { return averageFPS; }
    double getMinFPS() const { return minFPS; }
    double getMaxFPS() const { return maxFPS; }
    int getFrameCount() const { return frameCount; }

    void reset() {
        frameTimes.clear();
        totalTime = 0.0;
        frameCount = 0;
        minFPS = 999.0;
        maxFPS = 0.0;
        lastTime = std::chrono::high_resolution_clock::now();
    }
};


class ImGuiFPSDisplay {
private:
    FPSCounter fpsCounter;
    std::deque<float> fpsHistory;
    bool showDemo;
    bool showMetrics;

    // 性能监控变量
    double gpuSortTime;
    double renderTime;
    double totalFrameTime;

    static constexpr int HISTORY_SIZE = 120; // 2秒的历史数据（60FPS）

public:
    ImGuiFPSDisplay() :
        showDemo(false),
        showMetrics(true),
        gpuSortTime(0.0),
        renderTime(0.0),
        totalFrameTime(0.0) {}

    bool initialize(GLFWwindow* window) {
        // 初始化ImGui
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

        // 设置样式
        ImGui::StyleColorsDark();

        // 初始化平台绑定
        if (!ImGui_ImplGlfw_InitForOpenGL(window, true)) {
            std::cerr << "Failed to initialize ImGui GLFW" << std::endl;
            return false;
        }

        if (!ImGui_ImplOpenGL3_Init("#version 330")) {
            std::cerr << "Failed to initialize ImGui OpenGL3" << std::endl;
            return false;
        }

        return true;
    }

    void cleanup() {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
    }

    void update() {
        fpsCounter.update();

        // 更新FPS历史
        fpsHistory.push_back(static_cast<float>(fpsCounter.getCurrentFPS()));
        if (fpsHistory.size() > HISTORY_SIZE) {
            fpsHistory.pop_front();
        }
    }

    void render() {
        // 开始新的ImGui帧
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        if (showMetrics) {
            renderMetricsWindow();
        }

        if (showDemo) {
            ImGui::ShowDemoWindow(&showDemo);
        }

        // 渲染ImGui
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    }

    void setPerformanceTimes(double sortTime, double drawTime, double frameTime) {
        gpuSortTime = sortTime;
        renderTime = drawTime;
        totalFrameTime = frameTime;
    }

private:
    void renderMetricsWindow() {
        ImGui::Begin("Performance Metrics", &showMetrics, ImGuiWindowFlags_AlwaysAutoResize);

        // FPS信息
        ImGui::Text("FPS Information:");
        ImGui::Separator();
        ImGui::Text("Current FPS: %.1f", fpsCounter.getCurrentFPS());
        ImGui::Text("Average FPS: %.1f", fpsCounter.getAverageFPS());
        ImGui::Text("Min FPS: %.1f", fpsCounter.getMinFPS());
        ImGui::Text("Max FPS: %.1f", fpsCounter.getMaxFPS());
        ImGui::Text("Frame Count: %d", fpsCounter.getFrameCount());

        // FPS历史图表
        if (!fpsHistory.empty()) {
            ImGui::Spacing();
            ImGui::Text("FPS History:");
            std::vector<float> tempPlotData(fpsHistory.begin(), fpsHistory.end());
            ImGui::PlotLines("##FPS", tempPlotData.data(), fpsHistory.size(), 0, nullptr, 0.0f, 120.0f, ImVec2(300, 80));
        }

        ImGui::Spacing();
        ImGui::Separator();

        // 性能细分
        ImGui::Text("Performance Breakdown:");
        ImGui::Text("GPU Sort Time: %.3f ms", gpuSortTime);
        ImGui::Text("Render Time: %.3f ms", renderTime);
        ImGui::Text("Total Frame Time: %.3f ms", totalFrameTime);

        // 进度条显示各部分耗时比例
        if (totalFrameTime > 0.0) {
            float sortRatio = gpuSortTime / totalFrameTime;
            float renderRatio = renderTime / totalFrameTime;

            ImGui::Spacing();
            ImGui::Text("Time Distribution:");
            ImGui::ProgressBar(sortRatio, ImVec2(-1, 0), "GPU Sort");
            ImGui::ProgressBar(renderRatio, ImVec2(-1, 0), "Rendering");
        }

        ImGui::Spacing();
        ImGui::Separator();

        // 控制按钮
        if (ImGui::Button("Reset Statistics")) {
            fpsCounter.reset();
            fpsHistory.clear();
        }

        ImGui::SameLine();
        if (ImGui::Button("Show Demo")) {
            showDemo = !showDemo;
        }

        ImGui::End();
    }
};
#endif