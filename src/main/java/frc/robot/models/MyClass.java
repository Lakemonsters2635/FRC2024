package frc.robot.models;


import com.google.gson.Gson;
import java.util.ArrayList;


class Detection{
    public String objectLabel;
    public double x;
    public double y;
    public double z;
    public double confidence;
    public double xa;
    public double ya;
    public double za;
}
class DetectionList extends ArrayList<Detection> {
    @Override
    public boolean add(Detection detec) {
        return super.add(detec);
    }
    @Override
    public Detection get(int index) {
        return super.get(index);
    }
    @Override
    public Detection remove(int index) {
        return super.remove(index);
    }
}
class DetectionHelper {
    public DetectionList yoloObjects;
    public DetectionList aprilTags;

    public DetectionHelper() {
        yoloObjects = new DetectionList();
        aprilTags = new DetectionList();
    }
    public Detection getNearestAprilTag() {
        if (aprilTags.size() > 0) {
            return aprilTags.get(0);
        }
        return null;
    }
    public Detection[] getNearestAprilTags(int count) {
        Detection[] detections = new Detection[count];
        if (count <= aprilTags.size()) {
            for (int i = 0; i < count; i++) {
                detections[i] = aprilTags.get(i);
            }
            return detections;
        } else {
            return null;
        }
    }
    public Detection getNearestYoloDetection() {
        if (yoloObjects.size() > 0) {
            return yoloObjects.get(0);
        }
        return null;
    }
    public Detection[] getNearestYoloDetections(int count) {
        Detection[] detections = new Detection[count];
        if (count <= yoloObjects.size()) {
            for (int i = 0; i < count; i++) {
                detections[i] = yoloObjects .get(i);
            }
            return detections;
        } else {
            return null;
        }
    }
    // Yaw difference is from the frame of reference of rotation
    // Get Y from the rotaion object of detection and translate to z rotation of robot spo we can directly feed into command to turn robot
    // Confirm that we are turning in correct direction
    public double getYawDifferenceFromDetectionRotation(Detection detection) {
        // The whole point of this functions is so that we know which direction to turn the robot
        return 0.0;
    }
    public double getDistanceFromDetection(Detection detection) {
        return detection.z;
    }
    public double getXFieldDistanceFromDetection(Detection detection) {
        // x not
        return 0.0;
    }
    private double getThetaYZField(Detection detection) {
        double camX = detection.x;
        double camZ = detection.z;
        double yCamAngle = detection.ya;
        double thetaYZ = Math.tanh(camZ/camX);
        double thetaYZField = 90.0 - yCamAngle - thetaYZ;

        return thetaYZField;
    }
    public double getYFieldAprilFromDetection(Detection detection) {
        double yField;

        double radius = getRadius(detection);
        double thetaYZField = getThetaYZField(detection);

        yField = Math.cos(thetaYZField) * radius;

        return yField;
    }
    public double getXFieldAprilFromDetection(Detection detection) {
        double xField;

        double radius = getRadius(detection);
        double thetaYZField = getThetaYZField(detection);

        xField = Math.sin(thetaYZField) * radius;

        return xField;
    }
    public double getRadius(Detection detection) {
        double camX = detection.x;
        double camZ = detection.z;
        
        double radius = Math.sqrt(Math.pow(camX, 2) + Math.pow(camZ, 2));
        return radius;
    }
    public void updateDetections(String detectionsString, Gson gson) {
        DetectionList gsonOut = gson.fromJson(detectionsString, DetectionList.class);
        
        // Seperate out the detections with rotation
        for (int i = 0; i < gsonOut.size(); i++) { // Maybe change later
            if (gsonOut.get(i).objectLabel.substring(0,3).equals("tag")) {
                aprilTags.add(gsonOut.get(i));
                // System.out.println("UpdateDetections(: found apriltag");
            } else {
                yoloObjects.add(gsonOut.get(i));
                // System.out.println("yolo object");
            }
        }
    }
}
public class MyClass {
    // INPUT WILL ALWAYS BE SORTED BY objectLabel AND THEN z (distance)
    public static String jsonTestString = "[{\"objectLabel\":\"tag36h11\", \"x\": 14.0, \"y\":-2.3, \"z\":36.5, \"confidence\": 1.0, \"xa\":0.0,\"ya\":2.0,\"za\":-2.0}, {\"objectLabel\":\"tag36h12\", \"x\": 14.0, \"y\":-2.3, \"z\":36.5, \"confidence\": 1.0, \"xa\":0.0,\"ya\":2.0,\"za\":-2.0}, {\"objectLabel\":\"note\", \"x\": 2, \"y\":8, \"z\":43, \"confidence\":0.78, \"xa\":0.0,\"ya\":0.0,\"za\":0.0}]";
    public static void main(String args[]) {
        
        //printJson(jsonTestString);
        //String[] jsonTestArray = jsonTestString;
        
       //System.out.println(json);
        Gson gson = new Gson(); // Or use new GsonBuilder().create();
        // Warning: must use try catch in case the user tries to access a Rotation object inside a plain Detection object
        // TODO: we can fix this by using a different architecture but that syntax is slightly more complicated
        // Before using rotation, make sure it is not a note detection
        // DetectionList gsonOut = gson.fromJson(jsonTestString, DetectionList.class); // deserializes json into target2
        // extract gson from json
        
        DetectionHelper help = new DetectionHelper();
        
        help.updateDetections(jsonTestString, gson);

        System.out.println("Tag Size: " + help.aprilTags.size());
        System.out.println("Yolo Size: " + help.yoloObjects.size());
        
        for (Detection i : help.aprilTags) {
            System.out.println(i.objectLabel);
        }
        for (Detection i : help.yoloObjects) {
            System.out.println(i.objectLabel);
        }

        System.out.println(help.getNearestAprilTag().objectLabel);
        System.out.println(help.getNearestYoloDetection().objectLabel);

        System.out.println(help.getRadius(help.getNearestAprilTag()));

        // output values

        // System.out.println(gsonOut.get(1).objectLabel);
        // System.out.println(gsonOut.get(0).objectLabel);
        // System.out.println(gsonOut.get(0).x);
        // System.out.println(gsonOut.get(0).y);
        // System.out.println(gsonOut.get(0).z);
        // System.out.println(gsonOut.get(0).confidence);
        // System.out.println(gsonOut.get(0).rotation.x);
        // System.out.println(gsonOut.get(0).rotation.y);
        // System.out.println(gsonOut.get(0).rotation.z);
        // System.out.println(gsonOut.get(2).objectLabel);
        // System.out.println(gsonOut.get(2).x);
        // System.out.println(gsonOut.get(2).y);
        // System.out.println(gsonOut.get(2).z);
        // System.out.println(gsonOut.get(2).confidence);
        // System.out.println(gsonOut.get(2).rotation.x);
        // System.out.println(gsonOut.get(2).rotation.y);
        // System.out.println(gsonOut.get(2).rotation.z);
    }
}