var URL = 'ws://192.168.179.103:9090';
// Connecting to ROS
var ros = new ROSLIB.Ros({
  url : URL
});

//判断是否连接成功并输出相应的提示消息到web控制台
ros.on('connection', function() {
  console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
  console.log('Connection to websocket server closed.');
});

// =========================
// function switch
// =========================

// 0: no task

// 1: calibrate markers
// 2: localize camera
// 3: scan material
// 4: scan structure
// 5: assembly
// 6: show deviation
var state_names = ["no task", "calibrate markers", "localize camera", 
                   "scan material", "scan structure", "assembly", 
                   "show deviation"];

var param_state = new ROSLIB.Param({
  ros : ros,
  name : '/f_state'
});

function switch_state(state)
{
  if (state >= 0 && state <= 6)
    param_state.set(state);
  // document.getElementById('state').innerHTML = 'current state: ' + state_names[state];
}

// -------------------------
// END function switch
// -------------------------

// =========================
// deviation
// =========================

var deviation = new ROSLIB.Topic({
  ros : ros,
  name : '/deviation',
  messageType : 'std_msgs/Float32'
});

deviation.subscribe(function(message) {
  document.getElementById('deviation').innerHTML = 'deviation: ' + message.data.toFixed(2);
  // listener.unsubscribe();
});

// -------------------------
// END deviation
// -------------------------

// =========================
// previous / next
// =========================

var UNIT_INDEX = 0;
var UNIT_MAX = 4;
var param_index = new ROSLIB.Param({
  ros : ros,
  name : '/draw_index'
});

function previous()
{
  if (UNIT_INDEX > 0)
    UNIT_INDEX -= 1;
  param_index.set(UNIT_INDEX);
  loginfo("Return to element " + UNIT_INDEX);
}

function next()
{
  if (UNIT_INDEX < UNIT_MAX)
    UNIT_INDEX += 1;
  param_index.set(UNIT_INDEX);
  loginfo("Move on to element " + UNIT_INDEX);
}

// -------------------------
// END previous / next
// -------------------------

// =========================
// relocate
// =========================

var param_relocate = new ROSLIB.Param({
  ros : ros,
  name : '/relocate'
});

var param_keep_relocate = new ROSLIB.Param({
  ros : ros,
  name : '/keep_relocate'
});

function relocate()
{
  param_relocate.set(true);
}

function keep_relocate()
{
  param_keep_relocate.set(true);
}

// -------------------------
// END relocate
// -------------------------

// =========================
// take image
// =========================

var num_image = 0;

function save_image()
{
  loginfo("image " + num_image + " saved!");
  num_image += 1;
}

function perform_task()
{
  loginfo("Calculating...");
  loginfo("Done!");
  num_image = 0;
}

// -------------------------
// END take image
// -------------------------

function loginfo(str)
{
  document.getElementById("log").innerHTML += ("<br/ >[INFO] "+str);
}

function subscribe()
{
}
function unsubscribe()
{
}

