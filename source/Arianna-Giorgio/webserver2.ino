#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPClient.h>


//const char* ssid = "BiblioWiFi";
const char* ssid = "Pi_AP";
const char* password = "Raspberry";
String webPage ;
IPAddress fixed_ip(192,168,42,19);
IPAddress dns_ip(8,8,8,8);
IPAddress gw_ip(192,168,42,1);

ESP8266WebServer server(80);

const int led = 13;
HTTPClient http;

void provaaiax()
{
   webPage="<html>  <head>  <script src=\"http://code.jquery.com/jquery-latest.js\"></script>  <script type=\"text/javascript\">  $(document).ready(function() {  $(\"form#iscrizione\").submit(function(){ ";
webPage+="  var nome = $(\"#nome\").val();  var cognome = $(\"#cognome\").val();  $.ajax({  type: \"GET\",  url: \"/risp\",  data: \"nome=\" + nome + \"&cognome=\" + cognome,";
webPage+="  dataType: \"html\",  success: function(risposta) {  $(\"div#risposta\").html(risposta);  },  error: function(){  alert(\"Chiamata fallita!!!\");  }  });  return false;  }); }); </script> ";
webPage+="  </head> <body>  <form id=\"iscrizione\"> <p> Inserisci il nome:<br/> <input type=\"text\" name=\"nome\" id=\"nome\"/> </p> <p> Inserisci il cognome:<br/> <input type=\"text\" name=\"cognome\" id=\"cognome\"/>";
webPage+=" </p> <p> <input type=\"submit\" value=\"invia\"></p></form> <div id=\"risposta\"></div>  </body> </html> ";
 server.send(200, "text/html",webPage ); 
  
  }
void handleRoot() {
  digitalWrite(led, 1);
  server.send(200, "text/plain", "hello from esp8266!");
  digitalWrite(led, 0);
}
void risposta() {
  digitalWrite(led, 1);
  Serial.println(server.arg(0));
  server.send(200, "text/plain", "parto verso "+server.arg(0));
  digitalWrite(led, 0);
}

void handleNotFound(){
  digitalWrite(led, 1);
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
  digitalWrite(led, 0);
}

void setup(void){
  pinMode(led, OUTPUT);
  digitalWrite(led, 0);
  Serial.begin(115200);
  WiFi.config(fixed_ip, gw_ip, dns_ip);
  WiFi.begin(ssid, password);
  
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin("esp8266")) {
    Serial.println("MDNS responder started");
  }

  server.on("/", handleRoot);
  server.on("/aiax", provaaiax);
  server.on("/risp", risposta);
  server.on("/inline", [](){
    server.send(200, "text/plain", "this works as well");
  });

  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started");
}

void loop(void){
  http.begin("http://192.168.1.12/test.html"); //HTTP
  http.end();
  server.handleClient();
}
