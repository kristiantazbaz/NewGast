$(function() {
  var ros = new ROSLIB.Ros({
    url : 'ws://192.168.0.109:9090'
  });

  ros.on('connection', function() {
    console.log('Connected to websocket server.');
    $('#status span').html('Connected');
  });

  ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
    $('#status span').html('Error');
  });

  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
    $('#status span').html('Disconnected');
  });

  var moveSrvPasta = new ROSLIB.Service({
    ros : ros,
    name : '/pasta1/move',
    serviceType : 'gastronomous/MoveBy'
  });
  var moveSrvSauce = new ROSLIB.Service({
    ros : ros,
    name : '/sauce1/move',
    serviceType : 'gastronomous/MoveBy'
  });
  var mealSrv = new ROSLIB.Service({
    ros : ros,
    name : '/meal',
    serviceType : 'gastronomous/Meal'
  });
  var moveCb = function(result) {
    $('#alert-root').html(`
      <div class="alert alert-dismissible fade show" role="alert">
      <h4 class="alert-heading"></h4>
      <p id="alert-text"></p>
      <button type="button" class="close" data-dismiss="alert" aria-label="Close"><span aria-hidden="true">&times;</span></button>
      </div>
    `);
    if (result.success) {
      $('#alert-root div').addClass('alert-success');
      $('#alert-root h4').html('Success');
    } else {
      $('#alert-root div').addClass('alert-danger');
      $('#alert-root h4').html('Failed');
    }
    if (result.message.length > 0) {
      $('#alert-text').html(result.message);
    }
  };
  $('#test-form').on('submit', function (e) {
    e.preventDefault();
    var f = $(this);
    var moveReq = new ROSLIB.ServiceRequest({
      distance : parseInt(f.find('#distance').val(), 10),
      sleep : f.find('#sleep').is(':checked')
    });
    if (f.find('#pasta1').is(':checked')) {
      moveSrvPasta.callService(moveReq, moveCb);
    } else if (f.find('#sauce1').is(':checked')) {
      moveSrvSauce.callService(moveReq, moveCb);
    }
    return false;
  });

  $($('.pic-radio input')).change(function(e) {
    $(this).parents('.pic-radio').find('label').removeClass('btn-success');
    $(this).siblings().addClass('btn-success');
  });

  var topicId = undefined;
  var mealUpdateCb = function (msg) {
    console.log(msg);
    if (msg.id === topicId) {
      var r = $('#response-group li');
      var span = r.eq(msg.state).addClass('list-group-item-success').find('span');
      switch (msg.state) {
      case 0:
          span.html(msg.lineNum);
          break;
        case 2:
        case 4:
          cnt = new CountDownTimer(msg.stateTime);
          cnt.onTick(function(m, s){span.html(`${m}m${s}s`)}).start();
          break;
      }
      if (msg.complete) {
        topicId = undefined;
      }
    }
  };
  var mealsub = new ROSLIB.Topic({
    ros : ros,
    name : '/meal/update',
    messageType : 'gastronomous/MealUpdate'
  });
  mealsub.subscribe(mealUpdateCb);

  var mealCb = function (res) {
    topicId = res.topic;
    $('#response-group li').eq(0).addClass('list-group-item-success').find('span').html(res.lineNum);
  };

  $('#meal-form').on('submit', function (e) {
    e.preventDefault();
    $('#response-group li').removeClass('list-group-item-success');
    var f = $(this);
    var mealReq = new ROSLIB.ServiceRequest({
      pastaType : f.find('input[name=pasta-type]:checked').val(),
      pastaAmount : f.find('input[name=pasta-amount]:checked').val(),
      sauceType : f.find('input[name=sauce-type]:checked').val(),
      sauceAmount : f.find('input[name=sauce-amount]:checked').val(),
    });
    console.log(mealReq);
    mealSrv.callService(mealReq, mealCb);
    return false;
  });

});
