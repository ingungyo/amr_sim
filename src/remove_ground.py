#!/usr/bin/env python3
import open3d as o3d
import numpy as np
import math
import argparse

def remove_ground_ransac(
    input_path: str,
    output_path: str,
    distance_threshold: float = 0.05,
    ransac_n: int = 3,
    num_iterations: int = 200,
    eps_angle_deg: float = 25.0,
    axis: np.ndarray = np.array([0, 0, 1])
):
    """
    RANSAC으로 평면(지면)을 찾아 제거한 후, 남은 포인트를 저장한다.
    """

    print(f"[INFO] Loading: {input_path}")
    pcd = o3d.io.read_point_cloud(input_path)
    print(f"[INFO] Loaded {np.asarray(pcd.points).shape[0]} points")

    # 평면 추정
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=ransac_n,
        num_iterations=num_iterations,
    )

    [a, b, c, d] = plane_model
    normal = np.array([a, b, c])
    normal /= np.linalg.norm(normal)
    axis = axis / np.linalg.norm(axis)

    angle = math.degrees(math.acos(np.clip(np.dot(normal, axis), -1.0, 1.0)))
    print(f"[INFO] Plane model: {plane_model}")
    print(f"[INFO] Plane normal angle to axis: {angle:.2f}°")

    # 만약 평면이 너무 기울어져 있으면 "지면"이 아니라고 판단 (옵션)
    if angle > eps_angle_deg:
        print(f"[WARN] Plane angle {angle:.2f}° > {eps_angle_deg}°, not removing any points.")
        o3d.io.write_point_cloud(output_path, pcd)
        return

    # 인라이어(지면)와 아웃라이어(나머지) 분리
    ground = pcd.select_by_index(inliers)
    no_ground = pcd.select_by_index(inliers, invert=True)

    print(f"[INFO] Ground points: {len(inliers)}, Remaining: {len(no_ground.points)}")

    o3d.io.write_point_cloud(output_path, no_ground)
    print(f"[INFO] Saved filtered PCD: {output_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Remove ground plane using RANSAC (Open3D)")
    parser.add_argument("--input", required=True, help="Input PCD file path")
    parser.add_argument("--output", required=True, help="Output PCD file path")
    parser.add_argument("--distance", type=float, default=0.05, help="Distance threshold (m)")
    parser.add_argument("--angle", type=float, default=25.0, help="Max ground tilt angle (deg)")
    parser.add_argument("--iters", type=int, default=200, help="RANSAC iterations")
    args = parser.parse_args()

    remove_ground_ransac(
        input_path=args.input,
        output_path=args.output,
        distance_threshold=args.distance,
        eps_angle_deg=args.angle,
        num_iterations=args.iters,
    )
