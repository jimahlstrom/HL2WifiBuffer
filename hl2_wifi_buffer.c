// This software is Copyright (C) 2023-2024 by James C. Ahlstrom.
// This free software is licensed for use under the GNU General Public
// License (GPL), see http://www.opensource.org.
// Note that there is NO WARRANTY AT ALL.  USE AT YOUR OWN RISK!!

// This program reads the configuration file hl2_wifi_buffer.txt when it starts.
// Change your configuration there.

#include <arpa/inet.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdbool.h>
#include <pthread.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <net/if.h>
#include <ifaddrs.h>
#include <stdlib.h>

#define DEBUG	2

#define HTML_PORT	8080
#define BUFFER_SIZE	2048
#define NAME_SIZE	80
#define TX_BUF_BYTES	1038
#define TX_BUF_COUNT	(762 * 2)

#define TX_BUF_EMPTY	(txbuf_read == txbuf_write)
#define TX_BUF_FULL	(txbuf_write + 1 == txbuf_read || (txbuf_read == 0 && txbuf_write == txbuf_used - 1))

static int sock_hl2, sock_wifi_1024, sock_wifi_1025;
static int sock_listen;
static int delay;
static struct sockaddr_in sockaddr_in_client_1024, sockaddr_in_client_1025, sockaddr_in_hl2_1024, sockaddr_in_hl2_1025;
static struct in_addr hl2_hostaddr;
static struct in_addr wifi_hostaddr;
static double wifi_up_rate, wifi_down_rate;
static double wifi_jitter=0;
static unsigned int wifi_up_bytes, wifi_down_bytes;
static uint8_t num_receivers = 1;
static int sample_rate = 48000;
static int mox = 0;
static int txbuf_read = 0;
static int txbuf_write = 0;
static int txbuf_started = 0;
static int txbuf_used;
static unsigned int hl2_rx_samples = 0;
static unsigned int hl2_buffer_faults = 0;
static unsigned int wifi_buffer_faults = 0;
static char wifi_iface[NAME_SIZE + 4], hl2_iface[NAME_SIZE + 4];
static struct s_txbuf {
	short recv_len;
	uint8_t buf[TX_BUF_BYTES];
} TxBuf[TX_BUF_COUNT];
static pthread_mutex_t HL2_mutex = PTHREAD_MUTEX_INITIALIZER;

static double QuiskTimeSec(void)
{
	struct timespec ts;
#ifdef CLOCK_MONOTONIC_RAW
	if (clock_gettime(CLOCK_MONOTONIC_RAW, &ts) != 0)
#else
	if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
#endif
		return 0;
	return (double)ts.tv_sec + ts.tv_nsec * 1E-9;
}

static void * read_wifi_1024(void * arg)
{  // Data from WiFi that is copied to the HL2
	uint8_t * buffer;
	struct sockaddr_in addr;
	socklen_t sa_size;
	int i, j, recv_len;
	bool send_rqst;
	uint8_t C0_addr, speed;
	uint8_t seq[4];
	static double jitter;
	static double time_jitter = 0;
	static double time_rates = 0;
	double dtime;

	while (1) {
		// Read port 1024 from WiFi, place in TxBuf at index txbuf_write
		buffer = TxBuf[txbuf_write].buf;
		sa_size = sizeof(struct sockaddr_in);
		recv_len = recvfrom(sock_wifi_1024, buffer, TX_BUF_BYTES, 0, (struct sockaddr *)&addr, &sa_size);
		if (recv_len <= 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				//printf ("Read WiFi timeout\n");
				wifi_up_rate = wifi_down_rate = 0;
				wifi_up_bytes = wifi_down_bytes = 0;
				time_rates = time_jitter = QuiskTimeSec();
				wifi_jitter = jitter = 0;
			}
			else {
				perror("Read WiFi");
			}
			continue;
		}
		if (hl2_hostaddr.s_addr == 0 ||  addr.sin_addr.s_addr == hl2_hostaddr.s_addr)	// reject packet
			continue;
		wifi_up_bytes += recv_len + 18 + 20 + 8;
		TxBuf[txbuf_write].recv_len = recv_len;
		dtime = QuiskTimeSec();
		if (time_jitter == 0) {
			time_jitter = dtime;
			jitter = 0;
		}
		else {		// no lock
			if (dtime - time_jitter > jitter)
				jitter = dtime - time_jitter;
			time_jitter = dtime;
		}
		if (time_rates == 0) {
			time_rates = dtime;
		}
		else if (dtime - time_rates > 4.0) {		// no lock
			wifi_up_rate = wifi_up_bytes * 8.0 / (dtime - time_rates) / 1E6;
			wifi_down_rate = wifi_down_bytes * 8.0 / (dtime - time_rates) / 1E6;
			time_rates = dtime;
			wifi_up_bytes = wifi_down_bytes = 0;
			wifi_jitter = jitter;
			jitter = 0;
		}
		if (recv_len != 1032 && DEBUG > 1) {
			printf("WiFi1024 got %4d from %s port %d: ", recv_len, inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
			for (int i = 0; i < 10; i++)
				printf("%3X", buffer[i]);
			printf("\n");
		}
		sockaddr_in_client_1024 = addr;
		if (buffer[2] == 2 && buffer[0] == 0xEF && buffer[1] == 0xFE) {	// Discover Packet
			memset(&addr, 0, sizeof(struct sockaddr_in));
			addr.sin_family = AF_INET;
			addr.sin_port = htons(1024);
			inet_aton("169.254.255.255", &addr.sin_addr);
			if (sendto(sock_hl2, buffer, recv_len, 0, (struct sockaddr *)&addr, sizeof(struct sockaddr_in)) != recv_len)
				perror("Forward discover packet");
			continue;
		}
		else if (buffer[2] == 4 && buffer[0] == 0xEF && buffer[1] == 0xFE) {	// Start or Stop Packet
			pthread_mutex_lock(&HL2_mutex);
			num_receivers = 1;
			time_jitter = 0;
			time_rates = 0;
			sample_rate = 48000;
			txbuf_started = 0;
			txbuf_read = 0;
			txbuf_write = 0;
			mox = 0;
			hl2_buffer_faults = 0;
			wifi_up_bytes = wifi_down_bytes = 0;
			pthread_mutex_unlock(&HL2_mutex);
			if (sockaddr_in_hl2_1024.sin_addr.s_addr != 0)
				if (sendto(sock_hl2, buffer, recv_len, 0, (struct sockaddr *)&sockaddr_in_hl2_1024, sizeof(struct sockaddr_in)) != recv_len)
					perror("Forward Start/Stop to HL2");
			continue;
		}
		else if ( ! (recv_len == 1032 && buffer[3] == 0x02)) {	// Unknown packet - not I/Q Tx samples
			if (sockaddr_in_hl2_1024.sin_addr.s_addr != 0)
				if (sendto(sock_hl2, buffer, recv_len, 0, (struct sockaddr *)&sockaddr_in_hl2_1024, sizeof(struct sockaddr_in)) != recv_len)
					perror("Forward wifi to HL2");
			continue;
		}
		// This is the I/Q transmit samples from WiFi on endpoint 2.
		// Decode some needed data.
		C0_addr = (buffer[11] >> 1) & 0x3F;
		if (C0_addr == 0) {
			speed = buffer[12] & 0x03;
			num_receivers = ((buffer[15] >> 3) & 0x0F) + 1;
		}
		else {
			C0_addr = (buffer[523] >> 1) & 0x3F;
			if (C0_addr == 0) {
				speed = buffer[524] & 0x03;
				num_receivers = ((buffer[527] >> 3) & 0x0F) + 1;
			}
		}
		if (C0_addr == 0) {
			switch (speed) {
			case 0:
			default:
				sample_rate = 48000;
				break;
			case 1:
				sample_rate = 96000;
				break;
			case 2:
				sample_rate = 192000;
				break;
			case 3:
				sample_rate = 384000;
				break;
			}
			//printf("%d %d\n", num_receivers, sample_rate);
		}
		if (txbuf_used == 0) {		// Tx buffer is not in use - just copy packet
			mox = buffer[11] & 0x01;
			if (sockaddr_in_hl2_1024.sin_addr.s_addr != 0)
				if (sendto(sock_hl2, buffer, recv_len, 0, (struct sockaddr *)&sockaddr_in_hl2_1024, sizeof(struct sockaddr_in)) != recv_len)
					perror("Forward WiFi to HL2");
			continue;
		}
		if (buffer[11] & 0x80 || buffer[523] & 0x80)	// The RQST bit is set
			send_rqst = true;
		else
			send_rqst = false;
		pthread_mutex_lock(&HL2_mutex);
		if (TX_BUF_FULL) {
			wifi_buffer_faults++;
			if (DEBUG)
				printf("WiFi TxBuf overflow\n");
			txbuf_read = txbuf_write - txbuf_used / 2;
			if (txbuf_read < 0)
				txbuf_read += txbuf_used;
		}
		if (send_rqst) {	// correct the sequence numbers
			memcpy(seq, TxBuf[txbuf_read].buf + 4, 4);
			i = txbuf_read;
			while (i != txbuf_write) {
				j = i + 1;
				if (j >= txbuf_used)
					j = 0;
				memcpy(TxBuf[i].buf + 4, TxBuf[j].buf + 4, 4);
				if (++i >= txbuf_used)
					i = 0;
			}
			memcpy(TxBuf[txbuf_write].buf + 4, seq, 4);
			hl2_rx_samples -= (504 / (num_receivers * 6 + 2)) * 2;	// total samples for each receiver from HL2
		}
		else {
			// transmit I/Q samples from WiFi are in TxBuf at txbuf_write; save them
			if (++txbuf_write >= txbuf_used) 
				txbuf_write = 0;
		}
		pthread_mutex_unlock(&HL2_mutex);
		if (send_rqst && sockaddr_in_hl2_1024.sin_addr.s_addr != 0) {
			// copy the prevailing mox bit to this out-of-order packet
			if (mox)
				TxBuf[txbuf_write].buf[11] |= 0x01;
			else
				TxBuf[txbuf_write].buf[11] &= 0xFE;
			if (sendto(sock_hl2, TxBuf[txbuf_write].buf, TxBuf[txbuf_write].recv_len, 0,
					(struct sockaddr *)&sockaddr_in_hl2_1024, sizeof(struct sockaddr_in)) != recv_len)
				perror("Forward RQST to HL2");
			uint8_t * pt = TxBuf[txbuf_write].buf + 523;
			if (DEBUG)
				printf("RQST 0x%X, 0x%X, 0x%X, 0x%X, 0x%X \n", *pt++ >> 1, *pt++, *pt++, *pt++, *pt++);
		}
	}
	return NULL;
}

static void * read_hl2(void * arg)
{  // Data from the HL2 that is copied to WiFi
	unsigned char buffer[BUFFER_SIZE];
	uint8_t tx_buffer[TX_BUF_BYTES];
	struct sockaddr_in addr;
	socklen_t sa_size;
	int recv_len, ratio, tx_len;
	uint8_t hl2_tx_fifo = 0;
	static uint8_t hl2_tx_state = 0;
	uint8_t C0_addr;

	while (1) {
		buffer[0] = 0;
		sa_size = sizeof(struct sockaddr_in);
		recv_len = recvfrom(sock_hl2, buffer, BUFFER_SIZE, 0, (struct sockaddr *)&addr, &sa_size);
		if (recv_len <= 0) {
			perror("Read HL2");
			continue;
		}
		if (hl2_hostaddr.s_addr == 0 ||  addr.sin_addr.s_addr == hl2_hostaddr.s_addr)	// reject broadcast packet
			continue;
		if (DEBUG > 1 && recv_len != 1032) {
			printf(" HL2 got %4d from %s:%d ", recv_len, inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
			for (int i = 0; i < 10; i++)
				printf("%3X", buffer[i]);
			printf("\n");
		}
		if (recv_len == 1032 && buffer[3] == 0x06) {
			// count the number of HL2 internal buffer errors
			C0_addr = (buffer[11] >> 3) & 0x0F;
			if (C0_addr == 0) {
				hl2_tx_fifo = buffer[14];
			}
			else {
				C0_addr = (buffer[523] >> 3) & 0x0F;
				if (C0_addr == 0) {
					hl2_tx_fifo = buffer[526];
				}
			}
			if (C0_addr == 0) {	// check the HL2 internal error bit
				switch (hl2_tx_state) {
				case 0:			// mox is zero.
				default:
					if (mox) {
						hl2_tx_state = 1;
						//printf ("       mox %d, state %d, fifo 0x%X\n", mox, hl2_tx_state, hl2_tx_fifo);
					}
					break;
				case 1:			// mox changed to 1
					if (mox == 0) {
						hl2_tx_state = 0;
						//printf ("       mox %d, state %d, fifo 0x%X\n", mox, hl2_tx_state, hl2_tx_fifo);
					}
					else if (hl2_tx_fifo & 0x7F) {	// check for samples in the HL2 Tx buffer
						hl2_tx_state = 2;
						//printf ("       mox %d, state %d, fifo 0x%X\n", mox, hl2_tx_state, hl2_tx_fifo);
					}
					break;
				case 2:			// initial samples are in the buffer
					if (mox == 0) {
						hl2_tx_state = 0;
						//printf ("       mox %d, state %d, fifo 0x%X\n", mox, hl2_tx_state, hl2_tx_fifo);
					}
					else if (hl2_tx_fifo == 0x80 || hl2_tx_fifo == 0xFF) {	// check the error bit
						hl2_buffer_faults++;
						hl2_tx_state = 3;
						if (DEBUG)
							printf ("HL2 internal fault: mox %d, state %d, fifo 0x%X\n", mox, hl2_tx_state, hl2_tx_fifo);
					}
					break;
				case 3:			// the error bit was set; wait for it to clear
					if (mox == 0) {
						hl2_tx_state = 0;
						//printf ("       mox %d, state %d, fifo 0x%X\n", mox, hl2_tx_state, hl2_tx_fifo);
					}
					else if ((hl2_tx_fifo & 0x80) == 0) {
						hl2_tx_state = 2;
						//printf ("       mox %d, state %d, fifo 0x%X\n", mox, hl2_tx_state, hl2_tx_fifo);
					}
					break;
				}
			}
		}
		if (ntohs(addr.sin_port) == 1025) {
			sockaddr_in_hl2_1025 = addr;
			wifi_down_bytes += recv_len + 18 + 20 + 8;
			if (sendto(sock_wifi_1025, buffer, recv_len, 0, (struct sockaddr *)&sockaddr_in_client_1025, sizeof(struct sockaddr_in)) != recv_len)
				perror("Forward 1025 from HL2");
			continue;
		}
		sockaddr_in_hl2_1024 = addr;
		wifi_down_bytes += recv_len + 18 + 20 + 8;
		if (sendto(sock_wifi_1024, buffer, recv_len, 0, (struct sockaddr *)&sockaddr_in_client_1024, sizeof(struct sockaddr_in)) != recv_len)
			perror("Forward 1024 from HL2");
		if (txbuf_used == 0)
			continue;
		// Send TxBuf samples to the HL2
		tx_len = 0;
		pthread_mutex_lock(&HL2_mutex);
		if (txbuf_started == 0) {
			hl2_rx_samples = 0;
			if (txbuf_write >= txbuf_used / 2) {
				txbuf_started = 1;
				if (DEBUG)
					printf ("txbuf Started\n");
			}
		}
		else {
			if (recv_len == 1032 && buffer[3] == 0x06) {	// match the WiFi sending rate to the HL2 sending rate
				hl2_rx_samples += (504 / (num_receivers * 6 + 2)) * 2;	// total samples for each receiver from HL2
				ratio = sample_rate / 48000;		// send rate is 48 ksps
				if (hl2_rx_samples / ratio >= 63 * 2) {
					if (TX_BUF_EMPTY) {
						wifi_buffer_faults++;
						if (DEBUG)
							printf("WiFi TxBuf underflow\n");
						txbuf_started = 0;
						txbuf_read = 0;
						txbuf_write = 0;
					}
					else {
						tx_len = TxBuf[txbuf_read].recv_len;	// send the buffer packet to the HL2
						memcpy(tx_buffer, TxBuf[txbuf_read].buf, tx_len);
						if (++txbuf_read >= txbuf_used)
							txbuf_read = 0;
						hl2_rx_samples -= 63 * 2 * ratio;
					}
				}
			}
		}
		pthread_mutex_unlock(&HL2_mutex);
		if (tx_len > 0) {
			mox = tx_buffer[11] & 0x01;
			if (sendto(sock_hl2, tx_buffer, tx_len, 0,
					(struct sockaddr *)&sockaddr_in_hl2_1024, sizeof(struct sockaddr_in)) != recv_len)
				perror("Forward TxBuf to HL2");
		}
	}
	return NULL;
}

static void * webserver(void * arg)
{
	int sock_accept;
	int valread, valwrite;
	int txbuf_fill;
	double util;
	static double time_rates = 0;
	double dtime;
	char toggle = 'S';
	char buffer[BUFFER_SIZE];
	char * resp1 = "HTTP/1.0 200 OK\r\n"
"Server: webserver-c\r\n"
"Content-type: text/html\r\n\r\n"
"<html>\r\n"
"<head>\r\n"
"	<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\r\n"
"	<meta http-equiv=\"refresh\" content=\"3\">\r\n"
"	<title>Hermes-Lite2 WiFi Buffer</title>\r\n"
"</head>\r\n"
"<style>\r\n"
"table, th, td {\r\n"
"  border:1px solid black;\r\n"
"}\r\n"
"</style>\r\n"
"<body>\r\n"
;

	char * resp2 =
"<h4>Hermes-Lite2 Wifi Buffer</h4>\r\n"
"<b>Hermes Lite</b>\r\n"
"<br>\r\n"
"HL2 Interface %s\r\n"
"<br>\r\n"
"Interface address %s\r\n"
"<br>\r\n"
"Internal buffer faults %d\r\n"
"<br>\r\n"
"<br>\r\n"
;

	char * resp3 =
"<b>WiFi</b>\r\n"
"<br>\r\n"
"WiFi Interface %s\r\n"
"<br>\r\n"
"WiFi Address %s\r\n"
"<br>\r\n"
"Rate up %.1lf Mbits/sec\r\n"
"<br>\r\n"
"Rate down %.1lf Mbits/sec\r\n"
"<br>\r\n"
"Jitter %.3lf %ceconds\r\n"
"<br>\r\n"
"<br>\r\n"
"<b>WiFi Buffer</b>\r\n"
"<br>\r\n"
"Delay milliseconds %d\r\n"
"<br>\r\n"
"Utilization %.1f%%\r\n"
"<br>\r\n"
"Faults %d\r\n"
"<br>\r\n"
;

	char * resp4 = 
"</body>\r\n"
"</html>\r\n"
;

	while (1) {
		sock_accept = accept(sock_listen, NULL, NULL);
		if (sock_accept < 0) {
			perror("webserver (accept)");
			continue;
		}
		// Read from the socket
		valread = read(sock_accept, buffer, BUFFER_SIZE - 1);
		buffer[valread] = '\0';
		if (strstr(buffer, "favicon.ico")) {
			shutdown(sock_accept, SHUT_RDWR);
			close(sock_accept);
			continue;
		}
		//printf("connection accepted %d\n", valread);
		//printf("%s\n", buffer);
		if (valread < 0) {
			perror("webserver (read)");
			shutdown(sock_accept, SHUT_RDWR);
			close(sock_accept);
			continue;
		}
		// Write to the socket
		txbuf_fill = txbuf_write - txbuf_read;	// no mutex; depends on atomic integers
		if (txbuf_fill < 0)
			txbuf_fill += txbuf_used;
		valwrite = write(sock_accept, resp1, strlen(resp1));
		if (valwrite < 0)
			perror("webserver (write)");
		if (txbuf_used > 0)
			util = (float)txbuf_fill / txbuf_used * 100.0;
		else
			util = 0.0;
		snprintf(buffer, BUFFER_SIZE, resp2,
			hl2_iface[0] ? hl2_iface : "None", hl2_hostaddr.s_addr ? inet_ntoa(hl2_hostaddr) : "None", hl2_buffer_faults);
		valwrite = write(sock_accept, buffer, strlen(buffer));
		if (valwrite < 0)
			perror("webserver (write)");
		if (toggle == 'S')
			toggle = 's';
		else
			toggle = 'S';
		snprintf(buffer, BUFFER_SIZE, resp3,
			wifi_iface, inet_ntoa(wifi_hostaddr),
			wifi_up_rate, wifi_down_rate, wifi_jitter, toggle,
			delay, util, wifi_buffer_faults);
		valwrite = write(sock_accept, buffer, strlen(buffer));
		if (valwrite < 0)
			perror("webserver (write)");
		valwrite = write(sock_accept, resp4, strlen(resp4));
		if (valwrite < 0)
			perror("webserver (write)");
		shutdown(sock_accept, SHUT_RDWR);
		close(sock_accept);
	}
	return NULL;
}

static void search_interfaces(int * delay, char wifi_iface[], char hl2_iface[], struct in_addr * wifi_hostaddr, struct in_addr * hl2_hostaddr)
{  // Find the wifi and hl2 interface names. Get the delay from the config file.
	struct ifaddrs * ifap, * p;
	FILE * fp;
	char * s;
	char line[BUFFER_SIZE];

	*delay = 300;
	wifi_iface[0] = '\0';
	wifi_hostaddr->s_addr = 0;
	hl2_iface[0] = '\0';
	hl2_hostaddr->s_addr = 0;
	// read the configuration file
	fp = fopen("hl2_wifi_buffer.txt", "r");
	if (fp) {
		while (1) {	// read all the lines
			s = fgets(line, BUFFER_SIZE, fp);
			if (s == NULL)
				break;
			if (line[0] == '#')
				continue;
			if (strlen(line) < NAME_SIZE) {
				sscanf(line, " hl2_interface = %s", hl2_iface);
				sscanf(line, " wifi_interface = %s", wifi_iface);
				sscanf(line, " buffer_milliseconds = %d", delay);
			}
		}
		fclose(fp);
	}
	// search the interfaces for the names and addresses
	if (getifaddrs(&ifap) == 0) {
		p = ifap;
		while(p) {
			if ((p->ifa_addr) && p->ifa_addr->sa_family == AF_INET &&
					p->ifa_flags & IFF_RUNNING && ! (p->ifa_flags & IFF_LOOPBACK)) {
				//printf ("Found AF_INET interface %s address %s\n", p->ifa_name, inet_ntoa((*(struct sockaddr_in *)(p->ifa_addr)).sin_addr));
				if (! wifi_iface[0] && p->ifa_name[0] == 'w')
					strncpy(wifi_iface, p->ifa_name, NAME_SIZE);
				if (strncmp(wifi_iface, p->ifa_name, NAME_SIZE) == 0) {
					*wifi_hostaddr = (*(struct sockaddr_in *)(p->ifa_addr)).sin_addr;
					//printf("Addr %s %s %s\n", wifi_iface, p->ifa_name, inet_ntoa(*wifi_hostaddr));
				}
				if (! hl2_iface[0] && p->ifa_name[0] == 'e')
					strncpy(hl2_iface, p->ifa_name, NAME_SIZE);
				if (strncmp(hl2_iface, p->ifa_name, NAME_SIZE) == 0) {
					*hl2_hostaddr = (*(struct sockaddr_in *)(p->ifa_addr)).sin_addr;
					//printf("Addr %s %s %s\n", hl2_iface, p->ifa_name, inet_ntoa(*hl2_hostaddr));
				}
			}
			p = p->ifa_next;
		}
		freeifaddrs(ifap);
	}
	else {
		perror("getifaddrs failed");
	}
}

int main()
{
	int one = 1;
	int recv_len;
	int dummy;
	char dummy_iface[NAME_SIZE + 4];
	struct in_addr dummy_hostaddr;
	struct sockaddr_in addr;
	pthread_t thr_wifi, thr_hl2, thr_webserver;
	struct timeval rtimeout = {1, 0};
	char buffer[BUFFER_SIZE];
	bool started = false;
	socklen_t sa_size;
	memset(&sockaddr_in_client_1024, 0, sizeof(sockaddr_in_client_1024));
	memset(&sockaddr_in_client_1025, 0, sizeof(sockaddr_in_client_1025));
	memset(&sockaddr_in_hl2_1024, 0, sizeof(sockaddr_in_hl2_1024));
	memset(&sockaddr_in_hl2_1025, 0, sizeof(sockaddr_in_hl2_1025));

	while (1) {	// wait for WiFi network to start; get interfaces and addresses
		search_interfaces(&delay, wifi_iface, hl2_iface, &wifi_hostaddr, &hl2_hostaddr);
		if (wifi_iface[0] && wifi_hostaddr.s_addr)
			break;
		if (DEBUG)
			printf("Searching WiFi interfaces\n");
		sleep(4);
	}
	// set the used buffer size according to the delay
	txbuf_used = (int)(2.0 * delay / 2.625 * 2.0 + 0.5);	// size of the buffer space is twice the delay; maintain half full
	if (txbuf_used <= 0)	// Don't use the Tx buffer. Just copy the packets.
		txbuf_used = 0;
	else if (txbuf_used < 30)	// 20 milliseconds minimum
		txbuf_used = 30;
	else if (txbuf_used > TX_BUF_COUNT)
		txbuf_used = TX_BUF_COUNT;
	if (DEBUG)
		printf("delay %d txbuf_used %d\n", delay, txbuf_used);
	if (DEBUG)
		printf("WiFi interface %s address %s\n", wifi_iface, inet_ntoa(wifi_hostaddr));
	// Create a TCP socket for HTML
	sock_listen = socket(AF_INET, SOCK_STREAM, 0);
	if (sock_listen == -1)
		perror("webserver (socket)");
	setsockopt(sock_listen, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(int));
	// Create the address to bind the socket to
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(HTML_PORT);
	addr.sin_addr = wifi_hostaddr;
	// Bind the socket to the address
	if (bind(sock_listen, (struct sockaddr *)&addr, sizeof(addr)) != 0)
		perror("webserver (bind)");
	// Listen for incoming connections
	if (listen(sock_listen, SOMAXCONN) != 0)
		perror("webserver (listen)");
	if (pthread_create(&thr_webserver, NULL, &webserver, NULL) != 0)
		perror("Can't create webserver thread");
	//Create two UDP sockets and one thread for the WiFi interface
	sock_wifi_1024 = socket(AF_INET, SOCK_DGRAM, 0);
	sock_wifi_1025 = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock_wifi_1024 < 0 || sock_wifi_1025 < 0) {
		perror("Failed to create WiFi UDP socket");
		exit(2);
	}
	setsockopt(sock_wifi_1024, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(int));
	setsockopt(sock_wifi_1025, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(int));
	if (setsockopt(sock_wifi_1024, SOL_SOCKET, SO_BROADCAST, (char*)&one, sizeof(one)) != 0)
		perror("setsockopt broadcast for sock_wifi_1024 failed");
	if (setsockopt(sock_wifi_1024, SOL_SOCKET, SO_RCVTIMEO, (char*)&rtimeout, sizeof(rtimeout)) != 0)
		perror("setsockopt timeout for sock_wifi_1024 failed");
	if (setsockopt(sock_wifi_1025, SOL_SOCKET, SO_BROADCAST, (char*)&one, sizeof(one)) != 0)
		perror("setsockopt broadcast for sock_wifi_1025 failed");
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(1024);
	addr.sin_addr.s_addr = INADDR_ANY;
	if (bind(sock_wifi_1024, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		close(sock_wifi_1024);
		perror("Failed to bind the WiFi 1024 socket");
	}
	addr.sin_port = htons(1025);
	if (bind(sock_wifi_1025, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		close(sock_wifi_1025);
		perror("Failed to bind the WiFi 1025 socket");
	}

	while (1) {
		if ( ! started) {	// wait for the interface to the HL2 to start and return the interface and address
			if (hl2_iface[0] && hl2_hostaddr.s_addr) {
				started = true;
				// Create a thread and a socket for the HL2
				if (DEBUG)
					printf("HL2 interface %s address %s\n", hl2_iface, inet_ntoa(hl2_hostaddr));
				sock_hl2 = socket(AF_INET, SOCK_DGRAM, 0);
				if (sock_hl2 < 0) {
					perror("Failed to create HL2 socket");
					exit(1);
				}
				setsockopt(sock_hl2, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(int));
				if (setsockopt(sock_hl2, SOL_SOCKET, SO_BROADCAST, (char*)&one, sizeof(one)) != 0)
					perror("setsockopt broadcast for sock_hl2 failed");
				memset(&addr, 0, sizeof(addr));
				addr.sin_family = AF_INET;
				addr.sin_port = htons(0);
				addr.sin_addr = hl2_hostaddr;
				if (bind(sock_hl2, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
					close(sock_hl2);
					perror("Failed to bind the HL2 socket");
				}
				if (pthread_create(&thr_wifi, NULL, &read_wifi_1024, NULL) != 0)
					perror("Can't create WiFi thread");
				if (pthread_create(&thr_hl2, NULL, &read_hl2, NULL) != 0)
					perror("Can't create HL2 thread");
			}
			else {
				if (DEBUG)
					printf("Searching HL2 interface\n");
				sleep(4);
				search_interfaces(&dummy, dummy_iface, hl2_iface, &dummy_hostaddr, &hl2_hostaddr);
				continue;
			}
		}
		// Accept incoming connections from WiFi port 1025
		sa_size = sizeof(struct sockaddr_in);
		recv_len = recvfrom(sock_wifi_1025, buffer, BUFFER_SIZE, 0, (struct sockaddr *)&addr, &sa_size);
		if (recv_len <= 0) {
			perror("Read WiFi 1025");
			continue;
		}
		if (hl2_hostaddr.s_addr == 0 ||  addr.sin_addr.s_addr == hl2_hostaddr.s_addr)	// reject packet
			continue;
		wifi_up_bytes += recv_len + 18 + 20 + 8;
		if (DEBUG > 1) {
			printf("WiFi1025 got %4d from %s:%d ", recv_len, inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
			for (int i = 0; i < 10; i++)
				printf("%3X", buffer[i]);
			printf("\n");
		}
		sockaddr_in_client_1025 = addr;
		if (buffer[2] == 2 && buffer[0] == 0xEF && buffer[1] == 0xFE) {	// Discover Packet
			memset(&addr, 0, sizeof(struct sockaddr_in));
			addr.sin_family = AF_INET;
			addr.sin_port = htons(1025);
			inet_aton("169.254.255.255", &addr.sin_addr);
			if (sendto(sock_hl2, buffer, recv_len, 0, (struct sockaddr *)&addr, sizeof(struct sockaddr_in)) != recv_len)
				perror("Forward discover packet port 1025");
		}
		else {
			if (sockaddr_in_hl2_1025.sin_addr.s_addr != 0)
				if (sendto(sock_hl2, buffer, recv_len, 0, (struct sockaddr *)&sockaddr_in_hl2_1025, sizeof(struct sockaddr_in)) != recv_len)
					perror("Forward wifi 1025 to HL2");
		}
	}
	return 0;
}
