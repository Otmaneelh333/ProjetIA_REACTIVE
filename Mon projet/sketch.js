let pursuer1, pursuer2;
let target;
let obstacles = [];
let vehicules = [];
let snakeMode = false;
let currentMode = "snake";
let enemies = [];



function preload(){
  imageobstacle = loadImage("iconnuage.png");
  imagevehicle = loadImage("fenix.png");
  imagebackround = loadImage("esp.jpg");
  imageEnemy = loadImage("fantome.png");
}

function setup() {
  createCanvas(windowWidth, windowHeight);
  pursuer1 = new Vehicle(100, 100, imagevehicle);
  pursuer2 = new Vehicle(random(width), random(height), imagevehicle);

  vehicules.push(pursuer1);
  //vehicules.push(pursuer2);

  // On cree un obstace au milieu de l'écran
  // un cercle de rayon 100px
  // TODO
  obstacles.push({
    pos: createVector(width / 2, height / 2),
    r: 60
  });
}

function draw() {
  // changer le dernier param (< 100) pour effets de trainée
  image(imagebackround, 0, 0, width+1300, height+600);

  target = createVector(mouseX, mouseY);

  // Dessin de la cible qui suit la souris
  // Dessine un cercle de rayon 32px à la position de la souris
  fill(255, 0, 0);
  noStroke();
  circle(target.x, target.y, 32);


  fill(255);
  textSize(16);
  textAlign(LEFT, TOP);
  text(`cliquer pour ajouter un obstacle | v: ajouter vehicle | f: ajouter 5 vehicles | s: snak | g: wander | d: débogage | e: ennemi | n: null  => Mode actif : ${currentMode}`, 10, 10);

  


  // dessin des obstacles
  // TODO
  obstacles.forEach(o => {
    imageMode(CENTER);
    image(imageobstacle, o.pos.x, o.pos.y, o.r * 2, o.r * 2);
  });



  enemies.forEach((enemy, index) => {
    enemy.update();      // Mise à jour de la position
    enemy.show();        // Affichage
  
    if (enemy.detectCollision(vehicules)) {
      enemies.splice(index, 1); // Supprime l'ennemi s'il est tué
    }
  });


  vehicules.forEach((v,s) => {
     // Comportement de base : éviter les obstacles
    
    

    // Comportements spécifiques au mode actif
    if (currentMode === "snake") {
      
        let steeringForce;
    
        
          v.r = 16;
          if (s === 0) {
            // on a affaire au premier véhicule ! il suit bien la cible
            // normale rouge
            steeringForce = v.arrive(target);
          } else {
            // on a affaire à un des autres véhicules
            // il suit le véhicule précédent
            let vehiculePrecedent = vehicules[s - 1];
            const distanceEntreVehicules = 60;
            steeringForce = v.seekCorrectionSnake(vehiculePrecedent.pos, true,distanceEntreVehicules);
          }
       // Mouvement serpent
      v.applyForce(steeringForce);
    } else if (currentMode === "wander") {
      v.maxForce=0.6;
      v.maxSpeed=2.5;
      v.boundaries();
      v.wander();
      
      v.update();
      v.edges();
    } else {
      // Comportement par défaut : aller vers la cible avec ralentissement
      let arriveForce = v.arrive(target);
      v.applyForce(arriveForce);
    }
    let avoidForce = v.avoid(obstacles);
    avoidForce.mult(1.3);

    v.applyForce(avoidForce);
    // Mise à jour et affichage
    v.update();
    v.show();
  });
}

function mousePressed() {
  enemies.forEach((enemy, index) => {
    let d = dist(mouseX, mouseY, enemy.pos.x, enemy.pos.y);
    if (d < enemy.r) {
      enemies.splice(index, 1); // Supprime l'ennemi
    }
  });


  // TODO : ajouter un obstacle de taille aléatoire à la position de la souris
  obstacles.push(new Obstacle(mouseX, mouseY, random(30, 80), "white"));
}

function keyPressed() {
  if (key === "v") {
    vehicules.push(new Vehicle(random(width), random(height), imagevehicle)); // Ajouter un véhicule
  } else if (key === "f") {
    for (let i = 0; i < 10; i++) {
      let v = new Vehicle(20, 300, imagevehicle);
      v.vel = new p5.Vector(random(1, 5), random(1, 5));
      vehicules.push(v); // Ajouter 10 véhicules
    }
  } else if (key === "s") {
    currentMode = "snake";
  } else if (key === "g") {
    currentMode = "wander";
  } else if (key === "d") {
    Vehicle.debug = !Vehicle.debug;
  } else if (key === "n") {
    currentMode = "null";
  } else if (key === "e") {
    let newEnemy = new Enemy(random(width), random(height), imageEnemy);
    enemies.push(newEnemy);
  }
}