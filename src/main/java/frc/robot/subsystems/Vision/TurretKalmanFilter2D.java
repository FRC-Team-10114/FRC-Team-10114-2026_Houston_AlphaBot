package frc.robot.subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class TurretKalmanFilter2D {
    // 狀態矩陣 xHat = [角度 (rad), 角速度 (rad/s)]^T
    private Matrix<N2, N1> xHat = new Matrix<>(Nat.N2(), Nat.N1());

    // 誤差協方差矩陣 P (2x2)
    private Matrix<N2, N2> P = new Matrix<>(Nat.N2(), Nat.N2());

    // 系統物理矩陣 (State-Space)
    private final Matrix<N2, N2> A; // 狀態轉移矩陣
    private final Matrix<N2, N2> Q; // 過程雜訊矩陣 (對物理模型的信心)
    private final Matrix<N1, N2> H; // 觀測矩陣 (我們只「看」得到角度，看不到速度)
    
    private final double dt = 0.02; // FRC 標準週期 20ms

    public TurretKalmanFilter2D() {
        // A 矩陣定義物理運動學：
        // 角度 = 舊角度 + 角速度 * dt
        // 角速度 = 舊角速度 (假設等速)
        A = new Matrix<>(Nat.N2(), Nat.N2());
        A.set(0, 0, 1.0); A.set(0, 1, dt);
        A.set(1, 0, 0.0); A.set(1, 1, 1.0);

        // H 矩陣定義感測器：我們只能測量到狀態的第 0 項 (角度)
        H = new Matrix<>(Nat.N1(), Nat.N2());
        H.set(0, 0, 1.0); H.set(0, 1, 0.0);

        // Q 矩陣：物理模型的雜訊 (編碼器的隨機遊走)
        // 數字越小，越信任物理模型預測
        Q = new Matrix<>(Nat.N2(), Nat.N2());
        Q.set(0, 0, 0.01); // 角度的過程雜訊
        Q.set(1, 1, 0.1);  // 角速度的過程雜訊

        // 初始化 P 矩陣為單位矩陣
        P.set(0, 0, 1.0);
        P.set(1, 1, 1.0);
    }

    /**
     * 步驟 1：預測 (Predict) - 放在 periodic() 裡，每 20ms 呼叫一次
     */
    public void predict() {
        // 數學: xHat = A * xHat
        xHat = A.times(xHat);

        // 數學: P = A * P * A^T + Q
        P = A.times(P).times(A.transpose()).plus(Q);
    }

    /**
     * 步驟 2：修正 (Update) - 當視覺有看到 Tag 時呼叫
     * @param visionAngleRad 視覺算出的絕對砲台角度
     * @param visionStdDevRad 視覺的動態標準差 (信任度)
     */
    public void updateWithVision(double visionAngleRad, double visionStdDevRad) {
        // R 矩陣 (1x1)：視覺的觀測雜訊
        Matrix<N1, N1> R = VecBuilder.fill(visionStdDevRad * visionStdDevRad);
        
        // y (1x1)：感測器測量值 (視覺角度)
        Matrix<N1, N1> y = VecBuilder.fill(visionAngleRad);

        // --- 卡爾曼濾波器核心矩陣運算 ---

        // 1. 計算卡爾曼增益 K = P * H^T * (H * P * H^T + R)^-1
        Matrix<N2, N1> K = P.times(H.transpose())
                            .times(H.times(P).times(H.transpose()).plus(R).inv());

        // 2. 修正狀態 xHat = xHat + K * (y - H * xHat)
        Matrix<N1, N1> innovation = y.minus(H.times(xHat)); // 測量值與預測值的殘差
        xHat = xHat.plus(K.times(innovation));

        // 3. 修正協方差 P = (I - K * H) * P
        Matrix<N2, N2> I = Matrix.eye(Nat.N2());
        P = I.minus(K.times(H)).times(P);
    }

    /**
     * 取得濾波後的最優角度
     */
    public double getEstimatedAngleRad() {
        return xHat.get(0, 0);
    }

    /**
     * 取得濾波後的最優角速度 (這是 1D 濾波器做不到的！)
     */
    public double getEstimatedVelocityRadPerSec() {
        return xHat.get(1, 0);
    }
}