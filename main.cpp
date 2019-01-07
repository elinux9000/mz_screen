#include <iostream>
#include <vector>
#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <time.h>
#include <iterator>
#include <dirent.h>
#include <sstream>

using namespace std;
size_t write_serial(int s, const char * cmd)
{
	int debug_level = 0;
	size_t n_written = 0;
	size_t to_write = strlen(cmd);
	if (debug_level > 5)
		cout << "Command=" << cmd << endl;
	while (n_written < to_write)
	{
		n_written += write(s, &cmd[n_written], to_write - n_written);

	}
	usleep(10000);
	printf("Wrote %s %d\n",cmd,n_written);
	return n_written;
}
int read_serial(int s, char* buf, int len)
{
	int debug_level = 0;
	memset(buf, '\0', len);
	int total_read = 0;
	int bytes_read = 0;

	do
	{
		usleep(1);
		bytes_read = read(s, &buf[total_read], len - total_read);
		if (debug_level > 5 && total_read > 0)
			cout << &buf[total_read] << endl;
		total_read += bytes_read;

	}
	while (bytes_read > 0 && total_read < len);

	/* Error Handling */
	if (total_read == 0)
	{
		cout << "Error reading: " << strerror(errno) << endl;
	}
	else
		cout <<"Read "<<total_read<<endl;
	return total_read;
}

int open_serial(const char *filename)
{

	int s = open(filename, O_RDWR | O_NOCTTY);
	if (s < 0)
	{
		cout << "Could not open " << filename << endl;

	}
	else
	{
		cout << "Configuring " << filename << endl;
		struct termios tty;
		memset(&tty, 0, sizeof tty);

		//Error Handling
		if (tcgetattr(s, &tty) != 0)
		{
			cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
		}

		//Set Baud Rate
		cfsetospeed(&tty, B115200);
		cfsetispeed(&tty, B115200);

		//Setting other Port Stuff
		tty.c_cflag &= ~PARENB;        		// Make 8n1
		tty.c_cflag &= ~CSTOPB;
		tty.c_cflag &= ~CSIZE;
		tty.c_cflag |= CS8;
		tty.c_cflag &= ~CRTSCTS;       		// no flow control
		tty.c_lflag = 0; 			// no signaling chars, no echo, no canonical processing
		tty.c_oflag = 0;                  	// no remapping, no delays
		tty.c_cc[VMIN] = 0;                  	// read doesn't block
		tty.c_cc[VTIME] = 1;                  	// 0.1 seconds read timeout

		tty.c_cflag |= CREAD | CLOCAL;     	// turn on READ & ignore ctrl lines
		tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);     // turn off s/w flow ctrl
		tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
		tty.c_oflag &= ~OPOST;              // make raw

		// Flush Port, then applies attributes
		tcflush(s, TCIFLUSH);

		if (tcsetattr(s, TCSANOW, &tty) != 0)
		{
			cout << "Error " << errno << " from tcsetattr" << endl;
		}
	}
	return s;

}

int discover(void)
{

	DIR *dir;
	struct dirent *ent;
	char buf[256];
	if ((dir = opendir("/dev/")) != NULL)
	{
		while ((ent = readdir(dir)) != NULL)
		{
			string filename = "/dev/";
			filename.append(ent->d_name);
			if (filename.find("USB") != (size_t) -1)
			{
				printf("Trying %s\n", ent->d_name);
				int device = open_serial(filename.c_str());
				if (device > 0)
				{
					write_serial(device, "$SYS.OUTPUT.DISABLE=0\n");
					//write_serial(device, "dump-cal\n");
					if (read_serial(device, buf, sizeof(buf)) > 0)
					{
						printf("%s", buf);
						return device;
					}
					close(device);
				}

			}

		}
	}
	else
	{
		perror("");
	}
	return 0;
}
void test(void)
{
	char buf[256];
	int r0 = open_serial("/dev/ttyUSB0");
	int w = open_serial("/dev/ttyUSB1");

	write_serial(r0, "$SYS.OUTPUT.DISABLE=0\r\n");
	usleep(10000);
	int len = read_serial(r0, buf, sizeof(buf));
	for (int i = 0; i < len; i++)
	{
		printf("%d:%c\n", buf[i], buf[i]);
	}
}
int main(int argc, char* argv[])
{
	const bool debug = false;
	const int devel = 0;
	char letter;
	int device;
	char buf[4096];
	string filename = "/dev/ttyUSB0";
	if (argc > 1)
	{
		filename = argv[1];
	}
	//while (1)
	{
		device = open_serial(filename.c_str());
		if (device < 0)
		{
			cout << "Could not open " << filename;
			return (EXIT_FAILURE);
		}

		//while (1)
		{
			write_serial(device, "$AW\012\015");
			if (read_serial(device, buf, sizeof(buf)) > 0)
				cout << buf;

		}
	}
}

/*

 */
