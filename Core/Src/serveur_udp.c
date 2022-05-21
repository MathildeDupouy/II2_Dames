/*
 * serveur_udp.c
 *
 *  Created on: Apr 26, 2022
 *      Author: Ourspalois/Théo Ballet
 *      inspired by controllerstech's udpserver.c
 */


 #include "lwip/opt.h"

#include "lwip/api.h"
#include "lwip/sys.h"

#include "serveur_udp.h"

#include "string.h"

static struct netconn *conn;
static struct netbuf *buf;
static ip_addr_t *addr;
static unsigned short port;
char msg[100];
char smsg[200];

/*-----------------------------------------------------------------------------------*/
/******* Envoie une réponse et enregistre le message reçu *******/

static void udp_thread(void *arg){

	err_t err, recv_err; //servirons a recevoir les erreurs potentielles
	struct pbuf *txBuffer;

	/* creation d'un identifiant de connexion */
	conn = netconn_new(NETCONN_UDP) ;

	if (conn != NULL){
		/* rattacher la connexion a un port avec un adresse IP disponible */
		//err = netconn_bind(conn, IP_ADDR_ANY, 7) ; // version inconnue
		err = netconn_bind(conn, NULL, 7) ;

		if(err == ERR_OK){
			/* On rentre dans la boucle de la connexion */
			while(1){
				// receive data in the buffer
				recv_err = netconn_recv(conn, &buf) ;
				if (recv_err == ERR_OK) { // check if the data is received correctly
					taskENTER_CRITICAL( ) ;
					addr = netbuf_fromaddr(buf); // get the addr or the client in the msg
					port = netbuf_fromport(buf); // get the port of the client

					strcpy(msg, buf->p->payload) ;// copie du message. ajouter ici une messagerie

					//ecriture du message a renvoyer
					int len = sprintf(smsg, "\" %s\"was send by the client \n", (char *) buf->p->payload);

					/* allocation de l'espace du msg a renvoyer */
					txBuffer = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
					/* copier de la donnée dans le buffer pour transmition */
					pbuf_take(txBuffer, smsg, len) ;

					/* pointer le buffer de la connextion sur txBuffer*/
					buf->p = txBuffer ;

					netconn_connect(conn, addr, port) ; // on se connecte au client

					netconn_send(conn, buf); // send the msg

					/* clearing the buffers, adresses and netbuf*/
					buf->addr.addr = 0 ;
					pbuf_free(txBuffer) ;
					netbuf_delete(buf) ;
					// on a bien servi la connexion on peut attendre un nouveau message.
					taskEXIT_CRITICAL( );

				}

			}


		} // pas besoin de else on detruit la connexion

	} else {
		netconn_delete(conn);
	}



}

/*-----------------------------------------------------------------------------------*/
/******* initialise le serveur en creant un thread FREERTOS *******/
void udpserver_init(void)
{
  TaskHandle_t serveur_handle;
  xTaskCreate(udp_thread, "udp_thread", DEFAULT_THREAD_STACKSIZE, NULL, osPriorityNormal, &serveur_handle ) ;
}


