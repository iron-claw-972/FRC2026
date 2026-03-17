let pathJSON
let autoJSON
let importedPath
let importedAuto
let reader = new FileReader()
let anotherReader = new FileReader()
let fileLoaded = false
let fileVisualized = false
let autoLoaded = false
let autoVisualized = false

let firstTimeInAutos = true


let pathMode = true

let Ystatus
let Xstatus

const canvas = document.getElementById("graph");
const ctx = canvas.getContext("2d");

const fieldWidth = 8.071326
// ctx.strokeStyle = "white";
ctx.lineWidth = 5

// ctx.beginPath();
// ctx.moveTo(0,0);
// ctx.lineTo(1650, 800);
// ctx.stroke()

document.getElementById("pathInput").addEventListener("change", async () => {
    [importedPath] = document.getElementById("pathInput").files
    reader.readAsText(importedPath)
    fileLoaded = true

    document.getElementById("visualizeBtn").style.visibility = "visible"
    document.getElementById("visualizeBtn").style.fontSize = ""
    document.getElementById("visualizeBtn").style.width = ""

    // setTimeout(() => {
    //     visualizePath()
    // }, 100);
})

document.getElementById("autoInput").addEventListener("change", async () => {
    [importedAuto] = document.getElementById("autoInput").files
    anotherReader.readAsText(importedAuto)
    autoLoaded = true

    document.getElementById("visualizeBtn2").style.visibility = "visible"
    document.getElementById("visualizeBtn2").style.fontSize = ""
    document.getElementById("visualizeBtn2").style.width = ""
})

function visualizePath() {
    if (fileVisualized == false && fileLoaded) {
        document.getElementById("downloadBtn").style.visibility = "visible"
        document.getElementById("downloadBtn").style.opacity = 1
        document.getElementById("downloadAllBtn").style.visibility = "visible"
        document.getElementById("downloadAllBtn").style.opacity = 1
        fileVisualized = true
    }
    ctx.strokeStyle = "white";
    let startX = (JSON.parse(reader.result).waypoints[0].anchor.x) * 100
    let startY = ((1 - (Number(JSON.parse(reader.result).waypoints[0].anchor.y) / fieldWidth)) * fieldWidth) * 100
    let endX = (JSON.parse(reader.result).waypoints[1].anchor.x) * 100
    let endY = ((1 - (Number(JSON.parse(reader.result).waypoints[1].anchor.y) / fieldWidth)) * fieldWidth) * 100
    ctx.clearRect(0, 0, canvas.width, canvas.height)

    ctx.beginPath();
    ctx.moveTo(startX, startY);
    ctx.lineTo(endX, endY);
    ctx.stroke()

    if (document.getElementById("horizontal").checked) {
        startX = ((1 - (Number(JSON.parse(reader.result).waypoints[0].anchor.x) / 16.5)) * 16.5) * 100
        endX = ((1 - (Number(JSON.parse(reader.result).waypoints[1].anchor.x) / 16.5)) * 16.5) * 100
    }
    if (document.getElementById("vertical").checked) {
        // ctx.strokeStyle = "#d934eba4";
        startY = (JSON.parse(reader.result).waypoints[0].anchor.y) * 100
        endY = (JSON.parse(reader.result).waypoints[1].anchor.y) * 100
    }

    if (startX < 825) {
        ctx.strokeStyle = "#5234eba4";
        // Xstatus = "Blue"
        // console.log(startX)
    } else if (startX >= 825) {
        ctx.strokeStyle = "#eb3434a4";
        // Xstatus = "Red"
    }

    ctx.beginPath();
    ctx.moveTo(startX, startY);
    ctx.lineTo(endX, endY);
    ctx.stroke()
}

function exportPath() {
    //"reader.result" is just the text file
    // let [file] = document.getElementById("pathInput").files
    // let reader = new FileReader()

    //X and Y are normal for this
    //  the X is [0, 16.5]
    //  the Y is  [0, 8]
    pathJSON = JSON.parse(reader.result)
    console.log(pathJSON)
    // document.getElementById("input").innerHTML = JSON.stringify(pathJSON)
    console.log("START X: " + pathJSON.waypoints[0].anchor.x)
    console.log("START Y: " + pathJSON.waypoints[0].anchor.y)
    console.log("END X: " + pathJSON.waypoints[1].anchor.x)
    console.log("END Y: " + pathJSON.waypoints[1].anchor.y)

    //normal X and Y point flipping
    // Loop through all waypoints
    for (let i = 0; i < pathJSON.waypoints.length; i++) {
        let waypoint = pathJSON.waypoints[i];
        
        if (document.getElementById("horizontal").checked) {
            waypoint.anchor.x = ((1 - (Number(waypoint.anchor.x) / 16.5)) * 16.5)
            
            // Update nextControl if it exists
            if (waypoint.nextControl != null) {
                waypoint.nextControl.x = ((1 - (Number(waypoint.nextControl.x) / 16.5)) * 16.5)
            }
            // Update prevControl if it exists
            if (waypoint.prevControl != null) {
                waypoint.prevControl.x = ((1 - (Number(waypoint.prevControl.x) / 16.5)) * 16.5)
            }
        }

        if (document.getElementById("vertical").checked) {
            waypoint.anchor.y = ((1 - (Number(waypoint.anchor.y) / fieldWidth)) * fieldWidth)
            
            // Update nextControl if it exists
            if (waypoint.nextControl != null) {
                waypoint.nextControl.y = ((1 - (Number(waypoint.nextControl.y) / fieldWidth)) * fieldWidth)
            }
            // Update prevControl if it exists
            if (waypoint.prevControl != null) {
                waypoint.prevControl.y = ((1 - (Number(waypoint.prevControl.y) / fieldWidth)) * fieldWidth)
            }
        }
    }

    // next stuff is for point towards zones
    for (let i = 0; i < pathJSON.pointTowardsZones.length; i++) {
        if (document.getElementById("horizontal").checked) {
            pathJSON.pointTowardsZones[i].fieldPosition.x = ((1 - (Number(pathJSON.pointTowardsZones[i].fieldPosition.x) / 16.5)) * 16.5)
        }
        if (document.getElementById("vertical").checked) {
            pathJSON.pointTowardsZones[i].fieldPosition.y = ((1 - (Number(pathJSON.pointTowardsZones[i].fieldPosition.y) / fieldWidth)) * fieldWidth)
        }

        if (document.getElementById("horizontal").checked && document.getElementById("vertical").checked == false) {
            pathJSON.pointTowardsZones[i].rotationOffset = Number(pathJSON.pointTowardsZones[i].rotationOffset) * -1
        } else if (document.getElementById("horizontal").checked && document.getElementById("vertical").checked) {
            pathJSON.pointTowardsZones[i].rotationOffset = Number(pathJSON.pointTowardsZones[i].rotationOffset)
        } else if (document.getElementById("horizontal").checked == false && document.getElementById("vertical").checked) {
            pathJSON.pointTowardsZones[i].rotationOffset = -Number(pathJSON.pointTowardsZones[i].rotationOffset)
        }
        console.log("point towards zone: " + i)
    }

    // now into the starting state and end state rotation craziness
    if (document.getElementById("horizontal").checked && document.getElementById("vertical").checked == false) {
        pathJSON.idealStartingState.rotation = 180 - Number(pathJSON.idealStartingState.rotation)
        pathJSON.goalEndState.rotation = 180 - Number(pathJSON.goalEndState.rotation)
    } else if (document.getElementById("horizontal").checked && document.getElementById("vertical").checked) {
        pathJSON.idealStartingState.rotation = 180 + Number(pathJSON.idealStartingState.rotation)
        pathJSON.goalEndState.rotation = 180 + Number(pathJSON.goalEndState.rotation)
    } else if (document.getElementById("horizontal").checked == false && document.getElementById("vertical").checked) {
        pathJSON.idealStartingState.rotation = -Number(pathJSON.idealStartingState.rotation)
        pathJSON.goalEndState.rotation = -Number(pathJSON.goalEndState.rotation)
    }

    //this is for the mid-path rotations
    for (let i = 0; i < pathJSON.rotationTargets.length; i++) {
        if (document.getElementById("horizontal").checked && document.getElementById("vertical").checked == false) {
            pathJSON.rotationTargets[i].rotationDegrees = 180 - Number(pathJSON.rotationTargets[i].rotationDegrees)
        } else if (document.getElementById("horizontal").checked && document.getElementById("vertical").checked) {
            pathJSON.rotationTargets[i].rotationDegrees = 180 + Number(pathJSON.rotationTargets[i].rotationDegrees)
        } else if (document.getElementById("horizontal").checked == false && document.getElementById("vertical").checked) {
            pathJSON.rotationTargets[i].rotationDegrees = -Number(pathJSON.rotationTargets[i].rotationDegrees)
        }
        // console.log("rotation target: " + i)
    }

    console.log("CONVERTED START X: " + pathJSON.waypoints[0].anchor.x)
    console.log("CONVERTED START Y: " + pathJSON.waypoints[0].anchor.y)
    console.log("CONVERTED END X: " + pathJSON.waypoints[1].anchor.x)
    console.log("CONVERTED END Y: " + pathJSON.waypoints[1].anchor.y)

    if (pathJSON.waypoints[0].anchor.x < 8.25) {
        Xstatus = "Left"
    } else if (pathJSON.waypoints[0].anchor.x >= 8.25) {
        Xstatus = "Right"
    }
    if (pathJSON.waypoints[0].anchor.y < 4 && ((pathJSON.waypoints[1].anchor.y + pathJSON.waypoints[0].anchor.y) / 2) < 4) {
        Ystatus = "Bottom"
    } else if (pathJSON.waypoints[0].anchor.y >= 4 && ((pathJSON.waypoints[1].anchor.y + pathJSON.waypoints[0].anchor.y) / 2) >= 4) {
        Ystatus = "Top"
    } else {
        Ystatus = "bro what did you do for this to happen"
    }

    pathJSON.waypoints[0].linkedName = null
    pathJSON.waypoints[1].linkedName = null
    // document.getElementById("result").innerHTML = JSON.stringify(pathJSON)

    let pathMirror = new File(["\ufeff" + JSON.stringify(pathJSON)], Ystatus + Xstatus + " " + importedPath.name);
    document.getElementById("hiddenDownloader").href = window.URL.createObjectURL(pathMirror);
    document.getElementById("hiddenDownloader").download = pathMirror.name
    document.getElementById("hiddenDownloader").click()

}

function exportAllMirrors() {
    document.getElementById("vertical").checked = true
    document.getElementById("horizontal").checked = false
    exportPath()
    document.getElementById("vertical").checked = true
    document.getElementById("horizontal").checked = true
    exportPath()
    document.getElementById("vertical").checked = false
    document.getElementById("horizontal").checked = true
    exportPath()
    document.getElementById("vertical").checked = false
    document.getElementById("horizontal").checked = false

    alert(`Mirrored paths have been downloaded.\n \nMake sure to put these in your project's "pathplanner/paths" directory`)
}

function visualizeAuto() {
    document.getElementById("exportAuto").style.visibility = "visible"
    document.getElementById("exportAuto").style.opacity = 1

    document.getElementById("autoList").querySelectorAll("pre")[0].innerHTML = ""
    document.getElementById("autoList").querySelectorAll("pre")[1].innerHTML = ""

    for (let i = 0; i < (JSON.parse(anotherReader.result).command.data.commands.length); i++) {
        let checked
        let textColor
        console.log(i)

        document.getElementById("autoList").querySelectorAll("pre")[0].innerHTML += `
        `+ JSON.parse(anotherReader.result).command.data.commands[i].data.pathName + ``

        if (document.getElementById("bottomR").checked) {
            checked = "BottomRight"
            textColor = "#eb3434a4"
        } else if (document.getElementById("bottomL").checked) {
            checked = "BottomLeft"
            textColor = "#5234eba4"
        } else if (document.getElementById("topR").checked) {
            checked = "TopRight"
            textColor = "#fb0505a4"
        } else if (document.getElementById("topL").checked) {
            checked = "TopLeft"
            textColor = "#5234eba4"
        }
        document.getElementById("autoList").querySelectorAll("pre")[1].innerHTML += `<span style="color:` + textColor + `">
        `+ checked + " " + JSON.parse(anotherReader.result).command.data.commands[i].data.pathName + `</span>`

    }
    if (autoVisualized == false) {
        autoVisualized = true
    }
}

function exportAuto() {
    autoJSON = JSON.parse(anotherReader.result)
    let checked

    for (let i = 0; i < (autoJSON.command.data.commands.length); i++) {

        if (document.getElementById("bottomR").checked) {
            checked = "BottomRight"
        } else if (document.getElementById("bottomL").checked) {
            checked = "BottomLeft"
        } else if (document.getElementById("topR").checked) {
            checked = "TopRight"
        } else if (document.getElementById("topL").checked) {
            checked = "TopLeft"
        }

        autoJSON.command.data.commands[i].data.pathName = checked + " " + autoJSON.command.data.commands[i].data.pathName
    }

    let autoMirror = new File(["\ufeff" + JSON.stringify(autoJSON)], checked + " " + importedAuto.name);
    document.getElementById("hiddenDownloader").href = window.URL.createObjectURL(autoMirror);
    document.getElementById("hiddenDownloader").download = autoMirror.name
    document.getElementById("hiddenDownloader").click()
}

let modeCooldown = false
function switchModes() {
    if (modeCooldown == false) {
        pathMode = !pathMode

        if (pathMode) {
            document.getElementById("centerDiv2").style.opacity = 0
            setTimeout(() => {
                document.getElementById("centerDiv2").style.visibility = "hidden"
                document.getElementById("centerDiv2").style.top = "1000%"
            }, 200);
            document.getElementById("centerDiv").style.visibility = "visible"
            document.getElementById("centerDiv").style.opacity = 1
        } else if (pathMode == false) {
            document.getElementById("centerDiv").style.opacity = 0
            setTimeout(() => {
                document.getElementById("centerDiv").style.visibility = "hidden"
            }, 200);
            document.getElementById("centerDiv2").style.top = "50%"
            document.getElementById("centerDiv2").style.visibility = "visible"
            document.getElementById("centerDiv2").style.opacity = 1
            if (firstTimeInAutos) {
                alert('This is EXTREMELY experimental and will not work on most autos. If you are using it and think MAYBE it will work, make sure the paths you are using with it are in the same corner and have already been exported from the path mirrorer (this literally just renames the paths in the auto file for you.)') //basically its for lazy people
                firstTimeInAutos = false
            }
        }
        modeCooldown = true
        setTimeout(() => {
            modeCooldown = false
        }, 1000);
    }
}