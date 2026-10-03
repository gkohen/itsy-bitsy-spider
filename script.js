const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');

let width = canvas.width = window.innerWidth;
let height = canvas.height = window.innerHeight;

const mouse = { x: width / 2, y: height / 2 };

window.addEventListener('resize', () => {
    width = canvas.width = window.innerWidth;
    height = canvas.height = window.innerHeight;
});

const handleMove = (e) => {
    if (e.touches) {
        mouse.x = e.touches[0].clientX;
        mouse.y = e.touches[0].clientY;
    } else {
        mouse.x = e.clientX;
        mouse.y = e.clientY;
    }
};
window.addEventListener('mousemove', handleMove);
window.addEventListener('touchmove', handleMove, { passive: true });

// Matrix text helper variables
const matrixChars = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ\$#@&%*+=<>?/[]{}";
const getRandomChar = () => matrixChars[Math.floor(Math.random() * matrixChars.length)];

const lerp = (a, b, n) => (1 - n) * a + n * b;
const dist = (x1, y1, x2, y2) => Math.hypot(x2 - x1, y2 - y1);
const angleBetween = (x1, y1, x2, y2) => Math.atan2(y2 - y1, x2 - x1);

// Math 2-Joint Matrix String Solver
function solveIK(rootX, rootY, targetX, targetY, len1, len2, flip = 1) {
    const d = dist(rootX, rootY, targetX, targetY);
    let tX = targetX, tY = targetY;
    if (d > len1 + len2) {
        const angle = angleBetween(rootX, rootY, targetX, targetY);
        tX = rootX + Math.cos(angle) * (len1 + len2 - 0.1);
        tY = rootY + Math.sin(angle) * (len1 + len2 - 0.1);
    }
    const baseAngle = angleBetween(rootX, rootY, tX, tY);
    const currentDist = dist(rootX, rootY, tX, tY);
    const cosAngle1 = (len1 * len1 + currentDist * currentDist - len2 * len2) / (2 * len1 * currentDist);
    const angle1 = Math.acos(Math.max(-1, Math.min(1, cosAngle1)));

    const joint1X = rootX + Math.cos(baseAngle + angle1 * flip) * len1;
    const joint1Y = rootY + Math.sin(baseAngle + angle1 * flip) * len1;

    return { jointX: joint1X, jointY: joint1Y, effectorX: tX, effectorY: tY };
}

// Function to draw text strings along calculated limb paths
function drawTextLine(startX, startY, endX, endY, fontSize) {
    ctx.font = `${fontSize}px monospace`;
    const distanceBetween = dist(startX, startY, endX, endY);
    const steps = Math.floor(distanceBetween / (fontSize * 0.7));
    const angle = angleBetween(startX, startY, endX, endY);

    for (let i = 0; i <= steps; i++) {
        const t = i / Math.max(1, steps);
        const x = lerp(startX, endX, t);
        const y = lerp(startY, endY, t);
        
        ctx.save();
        ctx.translate(x, y);
        ctx.rotate(angle + Math.PI / 2);
        ctx.fillText(getRandomChar(), 0, 0);
        ctx.restore();
    }
}

class MatrixLeg {
    constructor(parent, angleOffset, side) {
        this.parent = parent;
        this.angleOffset = angleOffset;
        this.side = side;

        this.len1 = 50;
        this.len2 = 70;
        this.shoulderX = 0; this.shoulderY = 0;
        this.footX = 0; this.footY = 0;
        this.targetFootX = 0; this.targetFootY = 0;
        this.startX = 0; this.startY = 0;

        this.stepProgress = 1;
        this.stepSpeed = 0.2;
        this.stepRadius = 70;
        this.reachExt = 95;
        this.currentLift = 0;
        this.init();
    }

    init() {
        this.updateShoulder();
        this.footX = this.shoulderX + Math.cos(this.parent.angle + this.angleOffset) * (this.len1 + this.len2 * 0.7);
        this.footY = this.shoulderY + Math.sin(this.parent.angle + this.angleOffset) * (this.len1 + this.len2 * 0.7);
        this.targetFootX = this.footX;
        this.targetFootY = this.footY;
    }

    updateShoulder() {
        const rx = Math.cos(this.parent.angle);
        const ry = Math.sin(this.parent.angle);
        const localOffsetX = Math.cos(this.angleOffset) * 20;
        const localOffsetY = Math.sin(this.angleOffset) * 15;
        this.shoulderX = this.parent.x + (localOffsetX * rx - localOffsetY * ry);
        this.shoulderY = this.parent.y + (localOffsetX * ry + localOffsetY * rx);
    }

    update(canStep) {
        this.updateShoulder();
        const idealAngle = this.parent.angle + this.angleOffset;
        const idealX = this.shoulderX + Math.cos(idealAngle) * this.reachExt;
        const idealY = this.shoulderY + Math.sin(idealAngle) * this.reachExt;

        if (dist(this.targetFootX, this.targetFootY, idealX, idealY) > this.stepRadius && this.stepProgress >= 1 && canStep) {
            this.startX = this.footX;
            this.startY = this.footY;
            this.targetFootX = idealX + Math.cos(this.parent.angle) * (this.parent.speed * 3);
            this.targetFootY = idealY + Math.sin(this.parent.angle) * (this.parent.speed * 3);
            this.stepProgress = 0;
        }

        if (this.stepProgress < 1) {
            this.stepProgress += this.stepSpeed;
            this.footX = lerp(this.startX, this.targetFootX, this.stepProgress);
            this.footY = lerp(this.startY, this.targetFootY, this.stepProgress);
            this.currentLift = Math.sin(this.stepProgress * Math.PI) * 20;
        } else {
            this.footX = this.targetFootX;
            this.footY = this.targetFootY;
            this.currentLift = 0;
        }
    }

    draw() {
        const solved = solveIK(this.shoulderX, this.shoulderY, this.footX, this.footY - this.currentLift, this.len1, this.len2, this.side);

        // Subtle matrix shadow trace lines
        ctx.fillStyle = "rgba(0, 50, 20, 0.4)";
        drawTextLine(this.shoulderX, this.shoulderY + 10, solved.jointX, solved.jointY + 10, 10);
        drawTextLine(solved.jointX, solved.jointY + 10, solved.effectorX, solved.effectorY + 10, 10);

        // Core Glowing Matrix Text Strings
        ctx.fillStyle = "#00ff66";
        drawTextLine(this.shoulderX, this.shoulderY, solved.jointX, solved.jointY, 11);
        
        ctx.fillStyle = "#33ff88";
        drawTextLine(solved.jointX, solved.jointY, solved.effectorX, solved.effectorY, 10);

        // Matrix termination pulse
        ctx.fillStyle = "#ffffff";
        ctx.font = "12px monospace";
        ctx.fillText(getRandomChar(), solved.effectorX - 4, solved.effectorY - this.currentLift);
    }
}

class MatrixSpider {
    constructor(x, y) {
        this.x = x; this.y = y;
        this.angle = 0; this.speed = 0;
        this.legs = [
            new MatrixLeg(this, -Math.PI / 1.3, -1),
            new MatrixLeg(this, -Math.PI / 1.8, -1),
            new MatrixLeg(this, -Math.PI / 3.0, -1),
            new MatrixLeg(this, -Math.PI / 8.0, -1),
            new MatrixLeg(this, Math.PI / 1.3, 1),
            new MatrixLeg(this, Math.PI / 1.8, 1),
            new MatrixLeg(this, Math.PI / 3.0, 1),
            new MatrixLeg(this, Math.PI / 8.0, 1)
        ];
        this.gaitA = [0, 2, 5, 7];
        this.gaitB = [1, 3, 4, 6];
        this.activeGroup = true;
    }

    update() {
        const targetDist = dist(this.x, this.y, mouse.x, mouse.y);
        if (targetDist > 5) {
            const targetAngle = angleBetween(this.x, this.y, mouse.x, mouse.y);
            let diff = targetAngle - this.angle;
            while (diff < -Math.PI) diff += Math.PI * 2;
            while (diff > Math.PI) diff -= Math.PI * 2;
            this.angle += diff * 0.08;
            
            this.speed = Math.min(targetDist * 0.06, 6);
            this.x += Math.cos(this.angle) * this.speed;
            this.y += Math.sin(this.angle) * this.speed;
        } else {
            this.speed = 0;
        }

        let aReady = true, bReady = true;
        this.gaitA.forEach(i => { if (this.legs[i].stepProgress < 1) aReady = false; });
        this.gaitB.forEach(i => { if (this.legs[i].stepProgress < 1) bReady = false; });

        if (this.activeGroup && !aReady) this.activeGroup = false;
        else if (!this.activeGroup && !bReady) this.activeGroup = true;

        this.legs.forEach((leg, i) => {
            const isMyGroup = this.activeGroup ? this.gaitA.includes(i) : this.gaitB.includes(i);
            leg.update(isMyGroup);
        });
    }

    draw() {
        this.legs.forEach(leg => leg.draw());

        ctx.save();
        ctx.translate(this.x, this.y);
        ctx.rotate(this.angle);

        // Core Matrix Shell String Layering
        ctx.fillStyle = "#00ff66";
        ctx.font = "bold 13px monospace";
        ctx.fillText("[MATRIX_SYSTEM]", -45, -5);
        ctx.fillText(`X:${Math.round(this.x)}`, -20, 10);
        ctx.fillText(`Y:${Math.round(this.y)}`, -20, 22);

        // White hot terminal eyes
        ctx.fillStyle = "#ffffff";
        ctx.font = "bold 14px monospace";
        ctx.fillText("V", 20, -8);
        ctx.fillText("V", 20, 12);

        ctx.restore();
    }
}

const spider = new MatrixSpider(width / 2, height / 2);

function animate() {
    // Fade trail background to emulate CRT phosphorus glow delay
    ctx.fillStyle = "rgba(3, 5, 8, 0.15)";
    ctx.fillRect(0, 0, width, height);

    spider.update();
    spider.draw();

    requestAnimationFrame(animate);
}

animate();
