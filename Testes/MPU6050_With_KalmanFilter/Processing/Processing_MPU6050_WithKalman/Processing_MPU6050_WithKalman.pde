import processing.serial.*;

Serial myPort;

float pitch = 0;
float roll  = 0;

void setup() {
  size(800, 600, P3D);
  myPort = new Serial(this, "COM3", 115200); // <<< ajuste a porta serial
  myPort.bufferUntil('\n');
}

void draw() {
  background(30);
  lights();
  translate(width/2, height/2, 0);

  // Aplica rotações
  rotateX(radians(pitch));
  rotateZ(radians(roll));

  // Desenha "drone" como um cubo
  fill(0, 200, 200);
  stroke(255);
  box(150, 20, 150);

  // Desenha braços extras (só visual)
  strokeWeight(4);
  stroke(255, 0, 0);
  line(-100, 0, 0, 100, 0, 0); // eixo X
  stroke(0, 255, 0);
  line(0, -100, 0, 0, 100, 0); // eixo Y
}

void serialEvent(Serial p) {
  String inString = trim(p.readStringUntil('\n'));
  if (inString != null) {
    try {
      String[] parts = split(inString, " ");
      if (parts.length >= 2) {
        pitch = float(split(parts[0], ":")[1]);
        roll  = float(split(parts[1], ":")[1]);
      }
    } catch(Exception e) {
      println("Erro parse: " + inString);
    }
  }
}
