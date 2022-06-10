cette branche présente notre travail d'integration de LWIP sur STM32F746G 

*Ce travail n'a pas abouti a un jeu de dames fonctionnel * 

notre objectif de communication simple nous à amené a faire l'erreur de tenter une connection UDP, avec adresse IP fixe (on utilise pas DHCP, parce que bien plus simple sans). on choisi netconn, car la bibliothèque socket de LWIP nous parait vite obscure et pas beaucoup plus simlpe a debugger.

En effet notre conaissance du reseau est faible, le mode UDP nous paraissait plus simple que le TCP (ce qui est au final en partie faux), et surtout nous supposions que les pertes en connection seraient minimes et que les handshakes TCP n'auraient pas grand interet

nous tenons a souligner le travail du site controllerstech.com et de la personne derrière ce site qui a fourni des tutoriels très détaillés de mise en oeuvre de LWIP, nos codes (en particuliers ceux de debug sont grandement inspirés des siens)

tout d'abord pour tester l'ethernet on construit un serveur echo UDP qui va juste renvoyer sur le reseau les messages qu'il recoit. On utilise le logiciel gratuit Hercules comme interface ethernet Windows 
Cette démonstration s'avère fonctionnelle. 

Au moment de merger le code LWIP avec le jeux de dames plusieurs problèmes sont rencontrés. GCC ne trouve pas les librairies LWIP, l'adresse IP est mal ecrite par CubeMX (adresse trouvable dans /LWIP/App/lwip.c) 
et les fonctions LWIP ne se lancent pas. 

Le debug fut long : 
- le problèlme GCC vient d'une configuration sur mon PC qui empechait au compilateur d'acceder aux fichiers LWIP. 
- le problème d'adresse IP est connu des forum de ST, et il suffit juste d'arreter d'utiliser CubeMX (une fois LWIP enabled, tout se change dans le code) 7
* IL NE FAUT PAS GENERER DE CODE CUBE MX SUR LE PROJET* 
- la pile alouée a freertos était dépassée il a fallu l'augmenter, on se retrouvait avec un comportement alétoire caractéristique des stack overflow, et des taches non crées. 

finalement on se penche sur le code de la liaison appliquée au jeux de Dames, la structure proposée (et plutot simple a mettre en oeuvre meme si on ne l'a pas fait par manque de temps), consiste a ajouter un menu de selectiond de couleur des joueurs (on associé une adresse IP a chque couleur, puis lancé LWIPinit une fois le choix fait)
on aurait alors fait tourner un serveur et un client (deux connections LWIP sur deux ports différent, par convention on aurait put prendre les servs sur les ports7 et client sur port 8). 
Une carte pourrait alors envoyer les info via son port 8 sur le port 7 de l'autre carte a chaque coups. on aurait pas eut de problème de débit, en théorie tout aurait du fonctonner. 

Sauf que c'est à ce moment précis que lors de la redaction du code on se rend compte que les cartes ne communiquent pas. 

pour debugger on cree deux programmes : 
- un code serveur qui va faire clignoter une led a chaque message reçu. 
- un code client qui va evoyer un message indexé toutes les secondes. 

le code du serveur est le code actuel de la branche et est fonctionnel, il peut etre testé en envoyant des messages à 192.168.0.123 port 7.

le code client lui n'est pas focntionnel, enfin il fonctionne, mais envoie des messages en double, et en desordre. le code client est trouvable sur la branche ethenet_client. pour le fonctionneemnt il faut utiliser une IP fixe sur votre PC : 192.168.0.2 port 7, les messages viennt de 192.168.0.124 port 7.

un observation des paquets qui apssent sur le switch entre les deux cartes quand on les connecte montre que passé quelques messages les conexions s'arretent et c'est bien une erreur netconn que l'on retourve en debuggant la structure conn qui modelise nos connexions.

à ce stade j'ai passé beaucoup de temps a debugger et chercher et je ne peut pas continuer le projet

Je tiens a preciser que beaucoups de codes on des headers avec des noms de sites internet ou de contributeurs en lignes, mais ceux utilisés ont été modifié lourdement afin de faire les tests. 
