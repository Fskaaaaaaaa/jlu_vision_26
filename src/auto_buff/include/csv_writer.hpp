#pragma once
#include <fstream>
#include <string>
#include <vector>

class CsvWriter {
public:
    CsvWriter() = default;
    explicit CsvWriter(const std::string& path) {
        open(path);
    }

    ~CsvWriter() {
        close();
    }

    void open(const std::string& path) {
        close();
        ofs_.open(path, std::ios::out | std::ios::trunc);
    }

    bool isOpen() const {
        return ofs_.is_open();
    }

    void writeRow(const std::vector<std::string>& row) {
        if (!ofs_.is_open()) return;
        for (size_t i = 0; i < row.size(); ++i) {
            ofs_ << row[i];
            if (i + 1 < row.size()) ofs_ << ",";
        }
        ofs_ << "\n";
    }

    void close() {
        if (ofs_.is_open()) ofs_.close();
    }

private:
    std::ofstream ofs_;
};

// 写csv的
