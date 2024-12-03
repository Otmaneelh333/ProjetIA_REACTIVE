class Obstacle {
  constructor(x, y, r, couleur) {
    this.pos = createVector(x, y);
    this.r = r;
    this.color = couleur;
  }

  show() {
    push();
    fill(this.color);
    stroke(0)
    strokeWeight(1);
    ellipse(this.pos.x, this.pos.y, this.r * 2);
    fill(0);
    ellipse(this.pos.x, this.pos.y, 1);
    pop();
  }
}