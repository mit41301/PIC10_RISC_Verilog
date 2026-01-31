#include <fstream>
#include <iostream>
#include <string>
#include <iomanip>
#include <bitset>

using namespace std;

void line_handle(string buf, ofstream& out);

int main(int argc, char *argv[])
{
	if (argc != 2)
	{
		cout << "Input number of arguments not right. Only run this program with the full name of the hex file." << endl;
		system("pause");
		return 0;
	}
	ifstream input(argv[1], ios::in);
	ofstream output;
	string buf;

	output.open("program.hex", ios::out);

	if (!input.good())
	{
		cout << "Error: can not open this file" << endl;
		system("pause");
		return 0;
	}

	getline(input, buf);
	while (!input.eof())
	{
		cout << "Processing:" << endl;
		cout << buf << endl;
		line_handle(buf, output);
		getline(input, buf);
	}
	input.close();
	output.close();
	system("pause");
	return 0;
}

void line_handle(string buf, ofstream& out)
{
	if (buf == ":00000001FF")
	{
		out << ":00000001FF";
	}
	else
	{
		unsigned long byte_num = stoul(buf.substr(1, 2), 0, 16);
		unsigned long address = stoul(buf.substr(3, 4), 0, 16) / 2;
		for (unsigned int i = 9; i < 9 + byte_num * 2; i = i + 4)
		{
			unsigned long sum = 2 + (((address + (i - 9) / 4) % 256) + (address + (i - 9) / 4)/256) + stoul(buf.substr(i + 2, 2), 0, 16) + stoul(buf.substr(i, 2), 0, 16);
			out << ":02"; //Two byte to fit quartus
			out << setfill('0') << setw(4) << hex << uppercase << address + (i - 9)/4; //The address
			out << "00"; //Data
			out << buf.substr(i + 2, 2); //The lower byte to higher byte
			out << buf.substr(i, 2); //The higher byte to lower byte
			unsigned long temp = ((bitset<8>)sum).flip().to_ulong();
			out << setfill('0') << setw(2) << hex << uppercase << bitset<8>(temp + 1).to_ulong();
			out << endl;
		}
	}
}