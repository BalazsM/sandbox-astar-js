
// code based on:
//		https://github.com/CodingTrain/AStar

// --  globals  --------------------------------------------------------------

let simulationGridColumns = 10;
let simulationGridRows = 10;
let simulationGridSeed = 100;
let simulationGridWallRate = 0.3;
let simulationGridRegenerate = true;

let cellRadius;
let cellAngle;
let cellWidth;
let cellHeight;
let fillRadius;

let cellV0X;
let cellV0Y;
let cellV1X;
let cellV1Y;
let cellV2X;
let cellV2Y;
let cellV3X;
let cellV3Y;
let cellV4X;
let cellV4Y;
let cellV5X;
let cellV5Y;

let aStar;
let path;

let startX;
let startY;
let endX;
let endY;

let mouseGridX;
let mouseGridY;

// --  A* grid cell class  ---------------------------------------------------

class AStarGridCell {
	constructor (x, y) {
		this.x = x;
		this.y = y;
  
		this.worldX = 0;
		this.worldY = 0;
	  
		this.f = 0;
		this.g = 0;
  
		this.neighbors = [];
  
		this.previous = undefined;
  
		this.locked = false;
	}
  
	addNeighbors (grid, gridColumns, gridRows) {
	  const x = this.x;
	  const y = this.y;
  
	  if (x % 2) {
		if (y > 0)
		  this.neighbors.push(grid[x][y - 1]);
		
		if (x < gridColumns - 1 && y > 0)
		  this.neighbors.push(grid[x + 1][y - 1]);
		
		if (x < gridColumns - 1)
		  this.neighbors.push(grid[x + 1][y]);
  
		if (y < gridRows - 1)
		  this.neighbors.push(grid[x][y + 1]);
		
		if (x > 0)
		  this.neighbors.push(grid[x - 1][y]);
		
		if (x > 0 && y > 0)
		  this.neighbors.push(grid[x - 1][y - 1]);
	  } else {
		if (y > 0)
		  this.neighbors.push(grid[x][y - 1]);
  
		if (x < gridColumns - 1)
		  this.neighbors.push(grid[x + 1][y]);
  
		if (x < gridColumns - 1 && y < gridRows - 1)
		  this.neighbors.push(grid[x + 1][y + 1]);
  
		if (y < gridRows - 1)
		  this.neighbors.push(grid[x][y + 1]);
  
		if (x > 0 && y < gridRows - 1)
		  this.neighbors.push(grid[x - 1][y + 1]);
  
		if (x > 0)
		  this.neighbors.push(grid[x - 1][y]);
	  }
	}
}

// --  A* class  -------------------------------------------------------------

class AStar {
	constructor (gridColumns, gridRows) {
		this.grid = new Array(gridColumns);
		this.gridColumns = gridColumns;
		this.gridRows = gridRows;

		this.openSet = [];
		this.closedSet = [];

		this.startCell = null;
		this.endCell = null;

		for (let i = 0; i < this.gridColumns; i++) {
			this.grid[i] = new Array(this.gridRows);

			for (var j = 0; j < this.gridRows; j++) {
				this.grid[i][j] = new AStarGridCell(i, j);
			}
		}

		for (let i = 0; i < this.gridColumns; i++) {
			for (let j = 0; j < this.gridRows; j++) {
				this.grid[i][j].addNeighbors(this.grid, this.gridColumns, this.gridRows);
			}
		}
	}

	compute(startX, startY, endX, endY) {
		let result = null;
  
		for (let i = 0; i < this.gridColumns; i++) {
			for (var j = 0; j < this.gridRows; j++) {
				let cell = this.grid[i][j];
				cell.f = 0;
				cell.g = 0;
				cell.previous = undefined;
			}
		}

		this.startCell = aStar.grid[startX][startY];
		this.startCell.locked = false;
		this.endCell = aStar.grid[endX][endY];
		this.endCell.locked = false;
  
		this.openSet.length = 0;
		this.openSet.push(this.startCell);
		this.closedSet.length = 0;
	  
		let current = null; // TODO: rename to currentCell
	
	  	while (this.openSet.length > 0) {
			current = this.openSet[0];
			for (let cell of this.openSet) {
		  		if (cell.f < current.f) {
					current = cell;
		  		}
			}
		
			if (current === this.endCell)
				break;
  
			this.openSet.splice(this.openSet.indexOf(current), 1);
			this.closedSet.push(current);
  
			for (let neighbor of current.neighbors) {
				// Valid next spot?
				if (!this.closedSet.includes(neighbor) && !neighbor.locked) {
					let g = current.g + dist(neighbor.x, neighbor.y, current.x, current.y);
  
			// Is this a better path than before?
			let newPath = false;
			if (this.openSet.includes(neighbor)) {
			  if (g < neighbor.g) {
				neighbor.g = g;
				newPath = true;
			  }
			} else {
			  neighbor.g = g;
			  newPath = true;
			  this.openSet.push(neighbor);
			}
					// Yes, it's a better path
					if (newPath) {
						neighbor.f = neighbor.g + dist(neighbor.x, neighbor.y, this.endCell.x, this.endCell.y);
						neighbor.previous = current;
					}
				}
			}
		}
  
		if (current === this.endCell) {
			result = [];
			let temp = current;
			result.push(temp);
			while (temp.previous) {
				result.unshift(temp.previous);
				temp = temp.previous;
			}
		}
	  
		return result;
	}
}

// -- event handlers  --------------------------------------------------------

function setup() {
	let canvas = createCanvas(windowWidth, windowHeight);
	canvas.parent('workspace');

	smooth();
	
	startX = 0;
	startY = 0;

	updateAStar();

	cellV0X = cos(cellAngle * 0);
	cellV0Y = sin(cellAngle * 0);
	cellV1X = cos(cellAngle * 1);
	cellV1Y = sin(cellAngle * 1);
	cellV2X = cos(cellAngle * 2);
	cellV2Y = sin(cellAngle * 2);
	cellV3X = cos(cellAngle * 3);
	cellV3Y = sin(cellAngle * 3);
	cellV4X = cos(cellAngle * 4);
	cellV4Y = sin(cellAngle * 4);
	cellV5X = cos(cellAngle * 5);
	cellV5Y = sin(cellAngle * 5);

	updateGridConstants()
}

function windowResized() {
	resizeCanvas(windowWidth, windowHeight);

	updateGridConstants();
}
 
function mouseMoved() {
	const ox = cellWidth / 2;
	mouseGridX = abs(round((mouseX - ox) / cellWidth));

	const oy = cellRadius + ((mouseGridX % 2) ? 0 : cellHeight / 2);
	mouseGridY = abs(round((mouseY - oy) / cellHeight));

//	console.log(mouseGridX, mouseGridY, aStar.gridColumns, aStar.gridRows);

	if ((mouseGridX >= 0) &&
		(mouseGridX < aStar.gridColumns) &&
		(mouseGridY >= 0) &&
		(mouseGridY < aStar.gridRows)) {

		let doCompute = false;

		if (keyIsDown(32)) { // key = space
			aStar.grid[mouseGridX][mouseGridY].locked = !aStar.grid[mouseGridX][mouseGridY].locked;
			doCompute = true;
		}
		if (keyIsDown(87)) { // key = w
			aStar.grid[mouseGridX][mouseGridY].locked = 1;
			doCompute = true;
		}
		if (keyIsDown(65)) { // key = 65
			aStar.grid[mouseGridX][mouseGridY].locked = 0;
			doCompute = true;
		}
		if (keyIsDown(83)) { // key = s
			startX = mouseGridX;
			startY = mouseGridY;
			doCompute = true;
		}
		if (keyIsDown(69)) { // key = e
			endX = mouseGridX;
			endY = mouseGridY;
			doCompute = true;
		}

		if (doCompute) {
			path = aStar.compute(startX, startY, endX, endY);
		}
	}
}

function mousePressed() {
	if (mouseButton === LEFT) {
		if ((mouseGridX > 0) &&
			(mouseGridX < aStar.gridColumns) &&
			(mouseGridY > 0) &&
			(mouseGridY < aStar.gridRow)) {

			aStar.grid[mouseGridX][mouseGridY].locked = !aStar.grid[mouseGridX][mouseGridY].locked;
			path = aStar.compute(startX, startY, endX, endY);
		}
	}
}
  
function keyPressed() {
	mouseMoved();
}

function draw() {
	updateGlobals();

	if ((simulationGridColumns != aStar.gridColumns) ||
		(simulationGridRows != aStar.gridRows)) {
		updateAStar()
	}

	if (simulationGridRegenerate) {
		simulationGridRegenerate = false;
		generateGridWalls();
		path = aStar.compute(startX, startY, endX, endY);
	}

	clear();

	drawAStarGrid();

	drawPath(path);

	updateDisplays();
}

// --  gui sync  -------------------------------------------------------------

function updateGlobals() {
	simulationGridColumns = simulationGridColumnsInput.value * 1.0;
	simulationGridRows = simulationGridRowsInput.value * 1.0;
	simulationGridSeed = simulationGridSeedInput.value * 1.0;
	simulationGridWallRate = simulationGridWallRateInput.value * 1.0;
}

function updateDisplays() {
	simulationFPSDisplay.innerHTML = frameRate().toFixed(2);
	simulationStartPositionDisplay.innerHTML = `${startX}, ${startY}`;
	simulationEndPositionDisplay.innerHTML = `${endX}, ${endY}`;
	simulationPathLengthDisplay.innerHTML = path ? path.length : 'no solution';
}

// ---------------------------------------------------------------------------

function updateGridConstants() {
	cellRadius = (windowWidth / aStar.gridColumns) / 1.7;
	cellAngle = TWO_PI / 6;

	cellWidth = cellRadius * (1 + sin(cellAngle / 2));
	cellHeight = cellRadius * cos(cellAngle / 2) * 2;

	fillRadius = cellRadius - 2;
}

function updateAStar() {
	aStar = new AStar(simulationGridColumns, simulationGridRows);
	endX = aStar.gridColumns - 1;
	endY = aStar.gridRows - 1;

//	generateGridWalls();
	
//	path = aStar.compute(startX, startY, endX, endY);

	updateGridConstants();

	simulationGridRegenerate = true;
}

function generateGridWalls() {
	randomSeed(simulationGridSeed);
	let seeds = new Array(aStar.gridColumns);
	for (let i = 0; i < aStar.gridColumns; i++) {
		seeds[i] = random(1000);
	}

	for (let i = 0; i < aStar.gridColumns; i++) {
		randomSeed(seeds[i]);
		for (var j = 0; j < aStar.gridRows; j++) {
			aStar.grid[i][j].locked = (random(1) < simulationGridWallRate);
		}
	}
}

// --  draw functions  -------------------------------------------------------

function drawAStarGrid() {
	const cellLockedColor = color(180);
	const cellOpenColor = color(0, 255, 0, 50);
	const cellClosedColor = color(255, 0, 0, 50);

	for (let gridX = 0; gridX < aStar.gridColumns; gridX++) {
		for (let gridY = 0; gridY < aStar.gridRows; gridY++) {
			const cell = aStar.grid[gridX][gridY];

			let fillColor = null;
			if (cell.locked)
				fillColor = cellLockedColor;

			let strokeColor = null;
			if ((mouseGridX == gridX) && (mouseGridY == gridY))
				strokeColor = color(0);

			drawAStarGridCell(cell, fillColor, strokeColor);
		}
	}

	if (simulationShowOpenSet.checked) {
		for (let cell of aStar.openSet) {
			let strokeColor = null;
			if (mouseGridX == cell.x && mouseGridY == cell.y)
				strokeColor = color(0);

			drawAStarGridCell(cell, cellOpenColor, strokeColor);
		}
	}

	if (simulationShowClosedSet.checked) {
		for (let cell of aStar.closedSet) {
			let strokeColor = null;
			if (mouseGridX == cell.x && mouseGridY == cell.y)
				strokeColor = color(0);

			drawAStarGridCell(cell, cellClosedColor, strokeColor);
		}
	}
}

function drawAStarGridCell(cell, fillColor, strokeColor) {
	if (!fillColor && !strokeColor)
		return;

	if (fillColor)
		fill(fillColor);
	else
		noFill();

	if (strokeColor) {
		strokeWeight(3);
		stroke(200);
	} else {
		noStroke();
	}

	const ox = cellRadius;
	const oy = cellRadius + ((cell.x % 2) ? 0 : cellHeight / 2);

	beginShape();
	vertex(ox + cell.x * cellWidth + cellV0X * fillRadius, 
		oy + cell.y * cellHeight + cellV0Y * fillRadius);
	vertex(ox + cell.x * cellWidth + cellV1X * fillRadius, 
		oy + cell.y * cellHeight + cellV1Y * fillRadius);
	vertex(ox + cell.x * cellWidth + cellV2X * fillRadius, 
		oy + cell.y * cellHeight + cellV2Y * fillRadius);
	vertex(ox + cell.x * cellWidth + cellV3X * fillRadius, 
		oy + cell.y * cellHeight + cellV3Y * fillRadius);
	vertex(ox + cell.x * cellWidth + cellV4X * fillRadius, 
		oy + cell.y * cellHeight + cellV4Y * fillRadius);
	vertex(ox + cell.x * cellWidth + cellV5X * fillRadius, 
		oy + cell.y * cellHeight + cellV5Y * fillRadius);
	endShape(CLOSE);
}

function gridMap(gridX, gridY) {
	return {
		x : cellRadius + 
			gridX * cellWidth,
		y : cellRadius + ((gridX % 2) ? 0 : cellHeight / 2) +
			gridY * cellHeight
	};
}

function drawPath(path) {
//	const startColor = color(155, 116, 0, 200);
//	const endColor = color(255, 216, 0, 200);
	const startColor = color(0, 0, 255);
	const endColor = color(255, 0, 0);

	const ox = cellRadius;
	const oy = cellRadius + ((startX % 2) ? 0 : cellHeight / 2);

	let p1;
	let p2 = gridMap(startX, startY);

	noStroke();
	fill(startColor);
	ellipse(p2.x, p2.y, 6, 6);

	if (path) {
		strokeWeight(2);

		for (let i = 1; i < path.length; i++) {
			const cell = path[i];

			p1 = p2;
			p2 = gridMap(cell.x, cell.y);

			const color = lerpColor(startColor, endColor, i / path.length);
			stroke(color);
			line(p1.x, p1.y, p2.x, p2.y);

			noStroke();
			fill(color);
			ellipse(p2.x, p2.y, 6, 6);
		}
	}

	p2 = gridMap(endX, endY);

	noStroke();
	fill(endColor);
	ellipse(p2.x, p2.y, 6, 6);
}