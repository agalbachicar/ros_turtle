function onLoad(){
  //------------------------------------------------------------------------------------------------------
  // ROS Connection
  //------------------------------------------------------------------------------------------------------
  var ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
  });

  ros.on('connection', function() {
    var statusLbl = document.getElementById("statusLbl");
    statusLbl.innerHTML = 'Websocket connected.';
  });

  ros.on('error', function(error) {
    var statusLbl = document.getElementById("statusLbl");
    statusLbl.innerHTML = 'Error connecting to websocket.';
  });

  ros.on('close', function() {
    statusLbl.innerHTML = 'Server closed the connection.';
  });
  //------------------------------------------------------------------------------------------------------
  // Topic subscription
  //------------------------------------------------------------------------------------------------------
  var newPosition = function(newPosition){
    if (isLoaded !== false){
      var distance = getDistance(lastPosition, newPosition);
      if(distance > distanceError){
        positionLog(newPosition);
        //Convert postion to current canvas
        lastPositionTransformed = transformCoordinates(lastPosition); 
        newPositionTransformed = transformCoordinates(newPosition);
        //Draw the new line
        drawLine(lastPositionTransformed, newPositionTransformed);
        //Load the last position
        updateLastPosition(lastPosition);        
      }
    }
    else{
      isLoaded = true;
      //Load the last position
      updateLastPosition(lastPosition);  
    }
  };

  var listener = new ROSLIB.Topic({
    ros : ros,
    name : 'turtle1/pose',
    messageType : 'turtlesim/Pose'
  });
  listener.subscribe(function(message){
    newPosition(message);
  });



  //------------------------------------------------------------------------------------------------------
  // Canvas drawing utils
  //------------------------------------------------------------------------------------------------------
  var isLoaded = false;
  var lastPosition  = {
    x : 0.0,
    y : 0.0
  };

  var distanceError = 0.1;

  var updateLastPosition = function(position){
    var canvas = document.getElementById('turtleCanvas');
    lastPosition.x = position.x;
    lastPosition.y = position.y;
  }

  var drawLine = function (positionA, positionB){
    var canvas = document.getElementById('turtleCanvas');
     var ctx = canvas.getContext("2d");
     ctx.beginPath();
     ctx.moveTo(positionA.x, positionA.y);
     ctx.lineTo(positionA.x, positionB.y);
     ctx.stroke();
  };  

  var transformCoordinates = function(position){
    var canvas = document.getElementById('turtleCanvas');
    //var width = canvas.width;
    //var height = canvas.height;
    var width = 500.0;
    var height = 500.0
    return {
      x : (10.0 - position.x) / 10.0 * width,
      y : position.y / 10.0 * height
    };
  };  

  var positionLog = function(position){
    console.log('[' + position.x + ' ; ' + position.y + ' ]');
  };

  var getDistance = function(positionA, positionB){
    dx = positionB.x -positionA.x;
    dy = positionB.y -positionA.y;
    return Math.sqrt(dx * dx + dy * dy);
  }
};
