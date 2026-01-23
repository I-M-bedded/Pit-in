const ROSLIB = require('roslib');

var ros = new ROSLIB.Ros({ url: 'ws://192.168.1.30:9090' });
ros.on('connection', function () { console.log('Connected to websocket server.'); });
ros.on('error', function (error) { console.log('Error connecting to websocket server: ', error); });
ros.on('close', function () { console.log('Connection to websocket server closed.'); });


var topic = new ROSLIB.Topic({
   ros: ros,
   name: '/server_req_ver1',
   messageType: 'pitin_msgs/Srvop' 
});
/////305,1  257
var msg = new ROSLIB.Message({
   reqid_s: 49,
   seqid_s : 90,
   param1 : 0,
   param2 : 0, 
   param3 : 0.0,
   param4 : 0.0,
   param5 : "",
   param6 : "",
});

topic.publish(msg);





const http = require('http');

const hostname = '127.0.0.1';

const port = 3000;

const server = http.createServer((req, res) => {

  res.statusCode = 200;

  res.setHeader('Content-Type', 'text/plain');

  res.end('Hello World');

});




server.listen(port, hostname, () => {

  console.log(`Server running at http://${hostname}:${port}/`);

});