import time
import torch
from torch.utils.data import DataLoader
from ultralytics import YOLO
import os
import sys

# Project root setup
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
from vision.yolo.td_drp import TDDRPEndToEndPose
from vision.yolo.train_td_drp_pose import (
    PairedImageWithLabelsDataset, 
    collate_with_labels, 
    labels_to_batch_dict,
    extract_preds_for_loss,
    init_task_loss,
    TRAIN_CLEAN_DIR, 
    TRAIN_AUG_DIR, 
    TRAIN_LABEL_DIR,
    FOUNDATION_YOLO_WEIGHTS
)

def benchmark():
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    print(f"Full Loop Benchmarking on {device}")

    # Dataset
    ds2 = PairedImageWithLabelsDataset(TRAIN_CLEAN_DIR, TRAIN_AUG_DIR, TRAIN_LABEL_DIR)
    
    batch_sizes = [4, 6, 8]
    results = {}

    # Model Setup
    yolo = YOLO(FOUNDATION_YOLO_WEIGHTS)
    model = TDDRPEndToEndPose(yolo).to(device)
    model.set_phase(2)
    model.train()
    
    task_criterion = init_task_loss(model._yolo_detection_model)
    
    # Optimizer (to simulate weight updates)
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)

    for bs in batch_sizes:
        print(f"\nTesting Batch Size: {bs}")
        loader = DataLoader(
            ds2,
            batch_size=bs,
            shuffle=True,
            num_workers=4,
            pin_memory=True,
            collate_fn=collate_with_labels,
        )

        # Warm up (3 batches)
        it = iter(loader)
        for _ in range(3):
            try:
                batch = next(it)
            except StopIteration:
                it = iter(loader)
                batch = next(it)
        
        torch.cuda.synchronize()
        start_time = time.time()
        
        num_batches = 10
        processed_samples = 0
        
        for _ in range(num_batches):
            try:
                I_clean, I_aug, batch_labels = next(it)
            except StopIteration:
                it = iter(loader)
                I_clean, I_aug, batch_labels = next(it)
            
            I_clean = I_clean.to(device, non_blocking=True)
            I_aug = I_aug.to(device, non_blocking=True)
            
            optimizer.zero_grad()
            
            # Forward
            outputs = model(I_aug, I_clean=I_clean)
            
            # Loss Calculation (SSL + Task)
            ssl_kd_losses = model.criterion(
                F_aug_primes=outputs["F_primes"],
                F_cleans=outputs["F_cleans"],
                Z=outputs["Z"],
                I_aug=I_aug,
                delta_gammas=outputs["delta_gammas"],
                delta_betas=outputs["delta_betas"],
                expert_weights_list=model.get_expert_weights(),
                pose_logits_aug=outputs.get("pose_logits_aug"),
                pose_logits_clean=outputs.get("pose_logits_clean"),
            )
            
            task_loss = torch.tensor(0.0, device=device)
            if batch_labels.shape[0] > 0:
                preds = extract_preds_for_loss(outputs["x_aug"])
                batch_dict = labels_to_batch_dict(batch_labels, device)
                try:
                    raw_task_loss, _ = task_criterion(preds, batch_dict)
                    task_loss = raw_task_loss.sum() / I_aug.shape[0]
                except Exception:
                    pass
            
            total_loss = ssl_kd_losses["total"] + task_loss
            
            # Backward
            total_loss.backward()
            
            # Step
            optimizer.step()
            
            processed_samples += I_aug.shape[0]
            
        torch.cuda.synchronize()
        end_time = time.time()
        
        total_time = end_time - start_time
        samples_per_sec = processed_samples / total_time
        results[bs] = samples_per_sec
        print(f"Full Loop Throughput: {samples_per_sec:.2f} samples/sec")

    print("\n" + "="*40)
    print("Full Loop Benchmark Results (CPU+GPU)")
    print("="*40)
    for bs, speed in results.items():
        print(f"Batch Size {bs}: {speed:.2f} samples/sec")
    
    best_bs = max(results, key=results.get)
    print(f"\nRecommended Sweet Spot: {best_bs}")

if __name__ == "__main__":
    benchmark()
