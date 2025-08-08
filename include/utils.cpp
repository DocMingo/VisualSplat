#include "utils.h"
#include<map>
#include<format>
#include<fstream>

using namespace std;

/*
{
train_path:"D:\pointcloudSrc\train"
predict_path:"D:\pointcloudSrc\train\class2"
}
*/
std::map<std::string, std::string> parseFileData(const std::string& filePath) {
    // 使用 std::ifstream 读取文件，确保路径为 UTF-8 编码
    // std::ifstream file(filePath, std::ios::binary);
    std::ifstream file(filePath);
    if (!file) {
        string errorMsg = std::format("cannt open file: {}", filePath);
        throw std::runtime_error(errorMsg);
    }

    std::ostringstream ss;
    ss << file.rdbuf();
    string fileContent = ss.str();

    std::string startMarker = "{";
    std::string endMarker = "}";

    size_t startPos = fileContent.find(startMarker);
    size_t endPos = fileContent.find(endMarker);
    if (startPos == std::string::npos || endPos == std::string::npos) {
        string errorMsg = std::format("{} can't find relate data in file {}", errno, filePath);
        throw std::runtime_error(errorMsg);
    }

    string Data = fileContent.substr(startPos + 2, endPos - startPos + endMarker.length() - 3);
    cout << Data << endl;

    std::istringstream iss(Data);
    std::string line;
    std::vector<std::string> lines;

    while (std::getline(iss, line)) {
        lines.push_back(line);
    }

    // 将数据解析为键值对
    std::map<std::string, std::string> dataMap;
    for (const auto& line : lines) {
        cout << line << endl;
        size_t equalSignPos{ line.find(":") };
        if (equalSignPos != std::string::npos) {
            string key{ line.substr(0, equalSignPos) };
            string value{ line.substr(equalSignPos + 1) };
            key = key.substr(key.find_first_of(R"(")") + 1, key.find_last_of(R"(")") - 1);
            value = value.substr(value.find_first_of(R"(")") + 1, value.find_last_of(R"(")") - 1);
            // cout << key << " == " << value << endl;
            dataMap[key] = value;
        }
    }
    return dataMap;
}