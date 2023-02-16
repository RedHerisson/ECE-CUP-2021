#include "Connexion.h"

Connexion::Connexion()
{


}

int Connexion::initConnexionToServer()
{
	
	slen = sizeof(si_other);
	//Initialise winsock
	printf("\nInitialising Winsock...");
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		printf("Failed. Error Code : %d", WSAGetLastError());
		exit(EXIT_FAILURE);
	}
	printf("Initialised.\n");

	//create socket
	if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR)
	{
		printf("socket() failed with error code : %d", WSAGetLastError());
		exit(EXIT_FAILURE);
	}

	//setup address st2ructure
	memset((char*)&si_other, 0, sizeof(si_other));
	si_other.sin_family = AF_INET;
	si_other.sin_port = htons(PORT);
	si_other.sin_addr.S_un.S_addr = inet_addr(SERVER);
		
}

int Connexion::SendCommand(int parameter, int command){
	char commandChar[2];
	commandChar[0] = (char)command;
	commandChar[1] = (char)parameter;
	if (SHOW_DATA_SEND) {
		SYSTEMTIME st;
		GetLocalTime(&st);
		std::cout << std::setw(2) << st.wHour << ':'
			<< std::setw(2) << st.wMinute << ':'
			<< std::setw(2) << st.wSecond << '.'
			<< std::setw(3) << st.wMilliseconds ;
		cout << " ->" << "parametre : " << parameter << "// commande : " << command << "// size : " << strlen(commandChar) << endl;
		
	}
	if (sendto(s, commandChar, strlen(commandChar), 0, (struct sockaddr*)&si_other, slen) == SOCKET_ERROR)
	{
		printf("sendto() failed with error code : %d", WSAGetLastError());
		exit(EXIT_FAILURE);
	}
	
	return 1;
}
