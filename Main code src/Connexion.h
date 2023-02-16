#pragma once

#include "Header.hpp"

class Connexion
{
private :
	struct sockaddr_in si_other;
	int s;
	int slen;
	char buf[BUFLEN];
	char message[BUFLEN];
	WSADATA wsa;

public : 
	Connexion();
	int initConnexionToServer();
	int SendCommand(int command, int stateWanted);
};

