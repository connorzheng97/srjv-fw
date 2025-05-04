const char index_html[] PROGMEM = R"rawliteral(<!DOCTYPE HTML><html><head>
<title>SR-JV80 ESP32</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
</head><body>
<h2>SR-JV80 ESP32 by Connor Zheng</h2>
<h3>Main Menu</h3>
<a href="/rom">ROM content management</a><br>
<a href="/config">WiFi Configuration</a><br>
<a href="/ota">OTA Update</a><br><br>
<a href="/restart">Restart / Return to Low Power</a><br>
<h3>Firmware Version</h3>
<p>%fw_ver%</p>
</body></html>)rawliteral";

const char config_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>SR-JV80 ESP32</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head>
  <body>
  <h3>WiFi Configuration</h3>
  <form action="/get" target="hidden-form">
    SSID: <input type="text" name="ap_name" value="%ap_name%">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><p>SSID must be less or equal to 32 characters long.</p><br>
  <form action="/get" target="hidden-form">
    Password: <input type="password" name="ap_pass">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><p>Password must be at least 8 characters long.</p><br>
  <form action="/get" target="hidden-form">
  <label for="wifi_pwr_mode">Wireless power mode:</label>
  <select name="wifi_pwr_mode" id="wifi_pwr_mode">
    <option value="none" selected disabled hidden>%wifi_pwr_mode%</option>
    <option value="0">On</option>
    <option value="1">Off with Bluetooth Wakeup</option>
    <option value="2">All Off (Only Button Wakeup)</option>
  </select>
  <input type="submit" value="Submit" onclick="submitMessage()"></form>
  <p>WiFi can be turned on by pressing the button on the card or by using a phone to connect to the card's Bluetooth.</p><br>
  <form action="/get" target="hidden-form">
  <label for="ap_chan">Wi-Fi Channel:</label>
  <select name="ap_chan" id="ap_chan">
    <option value="none" selected disabled hidden>%ap_chan%</option>
    <option value="1">1</option>
    <option value="2">2</option>
    <option value="3">3</option>
    <option value="4">4</option>
    <option value="5">5</option>
    <option value="6">6</option>
    <option value="7">7</option>
    <option value="8">8</option>
    <option value="9">9</option>
    <option value="10">10</option>
    <option value="11">11</option>
  </select>
  <input type="submit" value="Submit" onclick="submitMessage()"></form>
  <p>Default channel is 11, recommended channel is 1, 6, or 11</p><br>
  <form action="/get" target="hidden-form">
  <label for="ap_hidden">Hide Wi-Fi SSID:</label>
  <select name="ap_hidden" id="ap_hidden">
    <option value="none" selected disabled hidden>%ap_hidden%</option>
    <option value="0">No</option>
    <option value="1">Yes</option>
  </select>
  <input type="submit" value="Submit" onclick="submitMessage()"></form>
  <p>Default is not hidden</p><br>
  <form action="/restart"><input type="submit" value="Restart ESP32"></form><br>
  <iframe style="display:none" name="hidden-form"></iframe>
</body>

<script>
  function submitMessage() {
    alert("A restart is required for change to take effect.");
    setTimeout(function(){ location.reload(); }, 100);   
  }
  </script>
</html>)rawliteral";

const char rom_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
<title>SR-JV80 ESP32</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
</head>
<body>
<h2>ROM Content Control</h2>
<h3>Current Status</h3>
<h4>Card Status: </h4>
<p id="status">%status%</p>
<h4>ROM ID (Address 0x00 to 0x1F): </h4>
<p id="rom_id">%rom_id%</p>
<h3>Erase Flash</h3>
<button id="eraseButton" onclick="eraseButton()" %erase_disabled%>Erase Flash</button>
<p id="eraseStatus"></p>
<p>Page will auto refresh when erase finish.<br>If page not refreshed after 180s, refresh manually.</p>
<h3>Program Flash</h3>
<form id="upload_form" enctype="multipart/form-data" method="post">
<input type="file" id="uploadPath" name="file1" onchange="validateSize(this)">
<input type="submit" id="uploadButton" name="upload" value="Upload" title = "Upload File" %program_disabled% onclick="uploadFile()"><br>
<progress id="progressBar" value="0" max="100" style="width:300px;"></progress>
<p id="uploadStatus"></p>
<p id="loaded_n_total"></p>
</form>
<p>The file must be bin format and less than or equal to 8MB (8388608).</p><br>
<form action="/restart"><input type="submit" id="restartButton" value="Restart / Return to Low Power"></form><br>
<p>If using multiple expansion cards it's strongly recommended to configure WiFi off in the configuration page.</p>
</body>

<script>
function _(el) {
  return document.getElementById(el);
}
function validateSize(input) {
  const fileSize = input.files[0].size; // in Bytes
  if (fileSize > 8388608) {
    alert('File size exceeds 8 MiB');
    input.value = null
  } else {
    // Proceed further
  }
}
function checkStatus() {
  var xhr = new XMLHttpRequest();
  xhr.onreadystatechange = function () {
  if (xhr.readyState == 4 && xhr.status == 200) {
    return xhr.responseText;}
  }
  xhr.open("GET", "/status", false);
  xhr.send();
  return xhr.onreadystatechange();
}
function uploadFile() {
  _("eraseButton").disabled = true;
  _("uploadButton").disabled = true;
  _("uploadPath").disabled = true;
  _("restartButton").disabled = true;
  _("status").innerHTML = "Programming";
  var file = _("uploadPath").files[0];
  var formdata = new FormData();
  formdata.append("uploadPath", file);
  var ajax = new XMLHttpRequest();
  ajax.upload.addEventListener("progress", progressHandler, false);
  ajax.addEventListener("load", completeHandler, false); // doesnt appear to ever get called even upon success
  ajax.addEventListener("error", errorHandler, false);
  ajax.addEventListener("abort", abortHandler, false);
  ajax.open("POST", "/upload");
  ajax.timeout=3600000;
  ajax.send(formdata);
}
function progressHandler(event) {
  _("loaded_n_total").innerHTML = "Uploaded " + event.loaded + " bytes";
  var percent = (event.loaded / event.total) * 100;
  _("progressBar").value = Math.round(percent);
  _("uploadStatus").innerHTML = Math.round(percent) + "&percnt; uploaded... please wait";
  if (percent >= 100) {
    _("uploadStatus").innerHTML = "Please wait";
    setInterval(function () {
    if (checkStatus() == "3"){
      location.reload(true);
  }
},1000);
  }
}
function completeHandler(event) {
  _("uploadStatus").innerHTML = "Upload Complete";
  _("progressBar").value = 0;
}
function errorHandler(event) {
  _("uploadStatus").innerHTML = "Upload Failed";
}
function abortHandler(event) {
  _("uploadStatus").innerHTML = "Upload Aborted";
}
function eraseButton() {
  _("eraseButton").disabled = true;
  _("uploadButton").disabled = true;
  _("uploadPath").disabled = true;
  _("restartButton").disabled = true;
  _("status").innerHTML = "Erasing";
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/erase", true);
  xhr.send();
  var seconds = 0;
  setInterval(function () {
  seconds = seconds + 1;
  _("eraseStatus").innerHTML = "Erasing, " + seconds + " seconds elapsed...";
  if (checkStatus() == "1"){ location.reload(true); }
},1000);
}
</script>

</html>
)rawliteral";

const char ota_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE html>
  <body style='width:480px'>
    <h2>ESP Firmware Update</h2>
    <form method='POST' enctype='multipart/form-data' id='upload-form'>
      <input type='file' id='file' name='update'>
      <input type='submit' value='Update'>
    </form>
    <br>
    <div id='prg' style='width:0;color:white;text-align:center'>0%</div>
  </body>
  <script>
    var prg = document.getElementById('prg');
    var form = document.getElementById('upload-form');
    form.addEventListener('submit', el=>{
      prg.style.backgroundColor = 'blue';
      el.preventDefault();
      var data = new FormData(form);
      var req = new XMLHttpRequest();
      var fsize = document.getElementById('file').files[0].size;
      req.timeout=60000;
      req.open('POST', '/update?size=' + fsize);
      req.upload.addEventListener('progress', p=>{
        let w = Math.round(p.loaded/p.total*100) + '%';
          if(p.lengthComputable){
             prg.innerHTML = w;
             prg.style.width = w;
          }
          if(w == '100%'){
            prg.style.backgroundColor = 'black';
            setTimeout(function(){ location.href = "/"; }, 5000);
          }
      });
      req.send(data);
     });
  </script>
)rawliteral";