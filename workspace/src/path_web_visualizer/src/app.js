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

  var pathClient = new ROSLIB.ActionClient({
    ros : ros,
    serverName : '/turtle_controller',
    actionName : 'path_server/PathAction'
  });

  document.getElementById('sendButton').onclick = function(){
    //Create goal
    var newGoal = createGoal();
    suscribeEventsToGoal(newGoal);
    newGoal.send();
  };

  var createGoal = function(){
    //Get the elements from the view, create the goal and transform the coordinates
    var newX = document.getElementById('newPositionX').value;
    var newY = document.getElementById('newPositionY').value;
    var position = {
        x: newX,
        y: newY
    };
    var transformedPosition = position;//transformCoordinatesBackFromWeb(position);
    return new ROSLIB.Goal({
      actionClient : pathClient,
      goalMessage : transformedPosition
    });  
  };

  var suscribeEventsToGoal = function(goal){
    goal.on('feedback', function(feedback) {
      var feedbackStatusText = 'Distancia completada ' + (feedback.progress * 100.0) + ' %';

      var coordiantes = transformCoordinatesToWeb(feedback.currentPosition);
      var positionText = '[' + coordiantes[0] + ';' + coordiantes[1] + ']';

      updateGoalFeedback(feedbackStatusText);
      updatePositionText(positionText);
    });

    goal.on('result', function(result) {
      if(result.rightPosition !== true){
        updateGoalFeedback('Goal rechazado');      
        return;
      }

      var feedbackStatusText = 'Distancia completada ' + (result.progress * 100.0) + ' %';

      var coordiantes = transformCoordinatesToWeb(result.currentPosition);
      var positionText = '[' + coordiantes[0] + ';' + coordiantes[1] + ']';

      updateGoalFeedback(feedbackStatusText);
      updatePositionText(positionText);    
    }); 
  };

  //------------------------------------------------------------------------------------------------------
  // Topic subscription
  //------------------------------------------------------------------------------------------------------
  var newPosition = function(newPosition){
    if (isLoaded !== false){
      var distance = getDistance(lastPosition, newPosition);
      if(distance > distanceError){
        //positionLog(newPosition);
        //Convert postion to current canvas
        lastPositionTransformed = transformCoordinatesToWeb(lastPosition); 
        newPositionTransformed = transformCoordinatesToWeb(newPosition);
        //Draw the new line
        drawLine(lastPositionTransformed, newPositionTransformed);
        //Load the last position
        updateLastPosition(newPosition);        
      }
    }
    else{
      isLoaded = true;
      //Load the last position
      updateLastPosition(newPosition);  
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

  var updateGoalFeedback = function(feedbackText){
    document.getElementById('goalFeedbackText').value = feedbackText;
  };
  var updatePositionText = function(positionText){
    document.getElementById('positionText').value = feedbackText;
  };

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
    lastPosition.x = position.x;
    lastPosition.y = position.y;
  }

  var drawLine = function (positionA, positionB){
    var canvas = document.getElementById('turtleCanvas');
     var ctx = canvas.getContext("2d");
     ctx.beginPath();
     ctx.moveTo(positionA.x, positionA.y);
     ctx.lineTo(positionB.x, positionB.y);
     ctx.stroke();
  };  

  var transformCoordinatesToWeb = function(position){
    var canvas = document.getElementById('turtleCanvas');
    //var width = canvas.width;
    //var height = canvas.height;
    var width = 500.0;
    var height = 500.0
    return {
      x : position.x / 10.0 * width,
      y : (10.0 - position.y) / 10.0 * height
    };
  };  

  var transformCoordinatesBackFromWeb = function(position){
    var canvas = document.getElementById('turtleCanvas');
    //var width = canvas.width;
    //var height = canvas.height;
    var width = 500.0;
    var height = 500.0
    return {
      x : position.x * 10.0 / width,
      y : 10.0 - (position.y * 10.0 / height)
    };
  }; 

  var positionLog = function(position){
    console.log('[' + position.x + ' ; ' + position.y + ' ]');
  };

  var getDistance = function(positionA, positionB){
    dx = positionB.x - positionA.x;
    dy = positionB.y - positionA.y;
    return Math.sqrt(dx * dx + dy * dy);
  }
};