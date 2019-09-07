import processing.io.*;
import gohai.glvideo.*;
import processing.serial.*;
Serial port;
GLCapture video;

color trackColor;

void setup() {
  size(640, 360, P2D);
  port = new Serial(this, "/dev/ttyACM0", 9600); 
  video = new GLCapture(this);
  video.start();
  trackColor = color(255, 0, 0);
  GPIO.pinMode(26, GPIO.OUTPUT);  //Kiri Maju
  GPIO.pinMode(20, GPIO.OUTPUT);  //Kiri Mundur
  GPIO.pinMode(6, GPIO.OUTPUT);   //Kanan Maju
  GPIO.pinMode(12, GPIO.OUTPUT);  //Kaman Mundur
}

void draw() {
   background(0);
  if (video.available()) {
    video.read();
  }
  
  video.loadPixels();
  image(video, 0, 0);
  
  float worldRecord = 500; 
  int closestX = 0;
  int closestY = 0;
  
  // Begin loop to walk through every pixel
  for (int x = 0; x < video.width; x ++ ) {
    for (int y = 0; y < video.height; y ++ ) {
      int loc = x + y*video.width;
      // What is current color
      color currentColor = video.pixels[loc];
      float r1 = red(currentColor);
      float g1 = green(currentColor);
      float b1 = blue(currentColor);
      float r2 = red(trackColor);
      float g2 = green(trackColor);
      float b2 = blue(trackColor);

      // Using euclidean distance to compare colors
      float d = dist(r1, g1, b1, r2, g2, b2); // We are using the dist( ) function to compare the current color with the color we are tracking.

      // If current color is more similar to tracked color than
      // closest color, save current location and current difference
      if (d < worldRecord) {
        worldRecord = d;
        closestX = x;
        closestY = y;
      }
    }
  }
  
  if (worldRecord < 10) { 
    // Draw a circle at the tracked pixel
    fill(trackColor);
    strokeWeight(4.0);
    stroke(0);
    ellipse(closestX, closestY, 16, 16);
    println(closestX,closestY);
    if (closestX<220)
    {
     port.write('0');
     GPIO.digitalWrite(26, GPIO.HIGH);
     GPIO.digitalWrite(20, GPIO.HIGH);
     GPIO.digitalWrite(6, GPIO.HIGH);
     GPIO.digitalWrite(12, GPIO.LOW);
     delay(10);
     GPIO.digitalWrite(26, GPIO.HIGH);
     GPIO.digitalWrite(20, GPIO.HIGH);
     GPIO.digitalWrite(6, GPIO.HIGH);
     GPIO.digitalWrite(12, GPIO.HIGH);   
     println("Turn Right"); 
    }
    else if (closestX>420)
    {
     port.write('0');
     GPIO.digitalWrite(26, GPIO.HIGH);
     GPIO.digitalWrite(20, GPIO.LOW);
     GPIO.digitalWrite(6, GPIO.HIGH);
     GPIO.digitalWrite(12, GPIO.HIGH);
     delay(10);
     GPIO.digitalWrite(26, GPIO.HIGH);
     GPIO.digitalWrite(20, GPIO.HIGH);
     GPIO.digitalWrite(6, GPIO.HIGH);
     GPIO.digitalWrite(12, GPIO.HIGH);
     println("Turn Left"); 
    }
    else if (closestY>1)
    {
     port.write('0');
     GPIO.digitalWrite(26, GPIO.HIGH);
     GPIO.digitalWrite(20, GPIO.LOW);
     GPIO.digitalWrite(6, GPIO.HIGH);
     GPIO.digitalWrite(12, GPIO.LOW);
     delay(10);
     GPIO.digitalWrite(26, GPIO.HIGH);
     GPIO.digitalWrite(20, GPIO.HIGH);
     GPIO.digitalWrite(6, GPIO.HIGH);
     GPIO.digitalWrite(12, GPIO.HIGH);
     println("Go Frwd"); 
    } 
    else
     {
     GPIO.digitalWrite(26, GPIO.HIGH);
     GPIO.digitalWrite(20, GPIO.HIGH);
     GPIO.digitalWrite(6, GPIO.HIGH);
     GPIO.digitalWrite(12, GPIO.LOW);
     delay(10);
     GPIO.digitalWrite(26, GPIO.HIGH);
     GPIO.digitalWrite(20, GPIO.HIGH);
     GPIO.digitalWrite(6, GPIO.HIGH);
     GPIO.digitalWrite(12, GPIO.HIGH);
     port.write('1');
     }   
  } 
  else
  {
     GPIO.digitalWrite(26, GPIO.HIGH);
     GPIO.digitalWrite(20, GPIO.HIGH);
     GPIO.digitalWrite(6, GPIO.HIGH);
     GPIO.digitalWrite(12, GPIO.LOW);
     delay(10);
     GPIO.digitalWrite(26, GPIO.HIGH);
     GPIO.digitalWrite(20, GPIO.HIGH);
     GPIO.digitalWrite(6, GPIO.HIGH);
     GPIO.digitalWrite(12, GPIO.HIGH);
     port.write('1');
  }
}

void mousePressed() {
  // Save color where the mouse is clicked in trackColor variable
  int loc = mouseX + mouseY*video.width;
  trackColor = video.pixels[loc];
}
