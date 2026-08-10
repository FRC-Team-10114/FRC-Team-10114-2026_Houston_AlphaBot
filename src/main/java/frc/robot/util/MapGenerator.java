package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class MapGenerator {
    
    // 這是一個標準的 Java Main 函式，可以在 VS Code 裡直接點擊右上角的 "Run Java" 執行
    public static void main(String[] args) {
        System.out.println("// ==========================================");
        System.out.println("// 🚀 ShotSolver 物理模擬生成的 20 點插值表");
        System.out.println("// ==========================================");

        // 設定範圍：從 1.0m 到 6.7m，總共 20 個點
        double startDist = 0.5;
        double endDist = 6.5;
        int points = 20;
        double step = (endDist - startDist) / (points - 1); 

        StringBuilder rollStr = new StringBuilder();
        StringBuilder hoodStr = new StringBuilder();
        StringBuilder tofStr = new StringBuilder();

        for (int i = 0; i < points; i++) {
            double dist = startDist + (i * step);

            // 模擬機器人在原點，完全靜止的狀態
            Pose3d robotPose = new Pose3d();
            Translation2d zeroVelocity = new Translation2d(0, 0);
            
            // 目標在正前方 dist 公尺處，Hub 高度為 1.8288 公尺
            Translation3d target3d = new Translation3d(dist, 0, 1.8288);

            // ⚡ 呼叫你的核心物理引擎進行疊代計算
            ShotSolver.Solution solution = ShotSolver.solve(
                    robotPose,
                    zeroVelocity,
                    0.0,
                    target3d,
                    ShotSolver.Calibration.identity()
            );

            if (solution.valid()) {
                // 將物理引擎的計算結果格式化成 Java 程式碼字串
                rollStr.append(String.format("rollMap.put(%.3f, RotationsPerSecond.of(%.2f));\n", dist, solution.flywheelRps()));
                hoodStr.append(String.format("hoodMap.put(%.3f, Degree.of(%.2f));\n", dist, Math.toDegrees(solution.hoodRad())));
                tofStr.append(String.format("timeOfFlightMap.put(%.3f, %.3f);\n", dist, solution.tofSec()));
            } else {
                System.err.println("⚠️ 警告：距離 " + dist + "m 解算失敗！可能超出仰角或轉速極限。");
            }
        }

        // 一次性印出所有結果，方便你直接複製貼上
        System.out.println("\n// 1. 飛輪轉速 (Flywheel RPS)");
        System.out.println(rollStr.toString());
        
        System.out.println("// 2. 仰角 (Hood Degree)");
        System.out.println(hoodStr.toString());
        
        System.out.println("// 3. 飛行時間 (Time of Flight)");
        System.out.println(tofStr.toString());
        
        System.out.println("// ==========================================");
    }
}