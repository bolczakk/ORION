#include <uxr/client/transport.h>

#include <rmw_microxrcedds_c/config.h>

#include "main.h"
#include "cmsis_os.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "app_conf.h"

// --- LWIP ---
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include <lwip/sockets.h>

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

// --- micro-ROS Transports ---
static int sock_fd = -1;
static unsigned open_users;
static uint8_t rx_packet[APP_UDP_MAX_PACKET + 1U];

bool cubemx_transport_reset(void) {
	if (sock_fd < 0) {
		open_users = 0;
		return true;
	}
	int fd = sock_fd;
	sock_fd = -1;
	open_users = 0;
	return closesocket(fd) == 0;
}

bool cubemx_transport_open(struct uxrCustomTransport *transport) {
	(void) transport;
	/* Ping can open the same transport while an RMW session is running. */
	if (sock_fd >= 0) {
		++open_users;
		return true;
	}

	struct sockaddr_in local = { 0 };
	struct sockaddr_in agent = { 0 };
	local.sin_family = AF_INET;
	local.sin_port = htons(APP_LOCAL_UDP_PORT);
	local.sin_addr.s_addr = htonl(INADDR_ANY);
	agent.sin_family = AF_INET;
	agent.sin_port = htons(APP_AGENT_PORT);
	agent.sin_addr.s_addr = inet_addr(APP_AGENT_IP);
	if (agent.sin_addr.s_addr == INADDR_NONE) {
		return false;
	}

	int fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (fd < 0) {
		return false;
	}
	if (bind(fd, (struct sockaddr *)&local, sizeof(local)) < 0 ||
	connect(fd, (struct sockaddr *)&agent, sizeof(agent)) < 0) {
		(void) closesocket(fd);
		return false;
	}
	sock_fd = fd;
	open_users = 1U;
	return true;
}

bool cubemx_transport_close(struct uxrCustomTransport *transport) {
	(void) transport;
	if (sock_fd < 0) {
		return true;
	}
	if (open_users > 1U) {
		--open_users;
		return true;
	}
	return cubemx_transport_reset();
}

size_t cubemx_transport_write(struct uxrCustomTransport *transport,
		const uint8_t *buf, size_t len, uint8_t *err) {
	(void) transport;
	if (err != NULL) {
		*err = 0;
	}
	if (sock_fd < 0 || buf == NULL || len == 0 || len > APP_UDP_MAX_PACKET) {
		if (err != NULL) {
			*err = 1;
		}
		return 0;
	}
	int count = send(sock_fd, buf, len, 0);
	if (count < 0 || (size_t) count != len) {
		if (err != NULL) {
			*err = 1;
		}
		return 0;
	}
	return (size_t) count;
}

size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf,
		size_t len, int timeout, uint8_t *err) {
	(void) transport;
	if (err != NULL) {
		*err = 0;
	}
	if (sock_fd < 0 || sock_fd >= FD_SETSIZE || buf == NULL || len == 0
			|| len > APP_UDP_MAX_PACKET || timeout < 0) {
		if (err != NULL) {
			*err = 1;
		}
		return 0;
	}

	fd_set read_fds;
	FD_ZERO(&read_fds);
	FD_SET(sock_fd, &read_fds);
	struct timeval wait = { .tv_sec = timeout / 1000, .tv_usec =
			(timeout % 1000) * 1000 };
	/* SO_RCVTIMEO=0 can mean infinite wait. select with zero really polls.
	 * Requires LWIP_SOCKET_SELECT=1 (the lwIP default). */
	int ready = select(sock_fd + 1, &read_fds, NULL, NULL, &wait);
	if (ready == 0) {
		return 0;
	}
	if (ready < 0) {
		if (errno != EINTR && err != NULL) {
			*err = 1;
		}
		return 0;
	}

	int count = recv(sock_fd, rx_packet, sizeof(rx_packet), MSG_DONTWAIT);
	if (count < 0) {
		if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR
				&& err != NULL) {
			*err = 1;
		}
		return 0;
	}
	/* Packet transport: never pass a truncated datagram to XRCE-DDS.
	 * One extra byte detects oversized datagrams even if recv truncates. */
	if ((size_t) count > len || (size_t) count > APP_UDP_MAX_PACKET) {
		if (err != NULL) {
			*err = 1;
		}
		return 0;
	}
	memcpy(buf, rx_packet, (size_t) count);
	return (size_t) count;
}

#endif
