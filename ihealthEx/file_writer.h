#pragma once
#include <fstream>
#include <string>
class FileWriter {
public:
	FileWriter() = default;
	FileWriter(const char *filename);
	void Initial();

	void WriteHeader();
	void WriteString(std::string s);

	void CloseFile();
private:
	std::string GetLocalTime();
private:
	std::fstream file;

};