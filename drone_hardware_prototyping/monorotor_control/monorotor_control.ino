struct Trajectory {
  double coeff[8];
  void display() {
    for (int i = 0; i < 8; i++) {
      Serial.print("coeff[");
      Serial.print(i);
      Serial.print("] = ");
      Serial.println(coeff[i]);
    }
  }
};

// Create empty instance of Trajectory struct
Trajectory traj; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}

void show() {
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command == "ON"){
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.println("LED is ON");
    } else if (command == "OFF") {
      digitalWrite(LED_BUILTIN, LOW);
      Serial.println("LED is OFF");
    } else if (command == "SHOW") {
      Serial.println("Show mode");
      show();
    } else if (command.startsWith("t")) {
      parseTrajectoryCommand(command);
      traj.display();
    }
    else {
      Serial.println("Unknown command");
    }
  }
}

void parseTrajectoryCommand(String command) {
  command = command.substring(1); // remove the initial 't'
  Serial.println(command); 
  // parse the command, storing numeric values as elements of traj 
  int index = 0;
  int start = 0;
  int end = command.indexOf(",");
  while(end != -1 && index < 8) {
    traj.coeff[index] = command.substring(start, end).toDouble();
    start = end + 1;
    end = command.indexOf(",", start);
    index++;
  }
  traj.coeff[index] = command.substring(start).toDouble();
}
