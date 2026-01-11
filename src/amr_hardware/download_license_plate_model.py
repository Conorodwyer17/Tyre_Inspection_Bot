#!/usr/bin/env python3
"""
Download Pre-trained License Plate Detection Model

This script downloads a pre-trained YOLOv8 model for license plate detection
from Hugging Face and saves it to the deep_learning_models directory.

Models available:
1. Koushim/yolov8-license-plate-detection (YOLOv8n)
2. RawanAlwadeya/YOLOCarAndPlateDetection (YOLOv8 - detects cars and plates)
3. morsetechlab/yolov11-license-plate-detection (YOLOv11)

Note: The system uses segmentation models, but detection models can work
with some modifications. We'll try to find a segmentation model or use
a detection model and adapt it.
"""

import os
import sys
from pathlib import Path
import subprocess

def install_dependencies():
    """Install required packages."""
    print("Installing dependencies...")
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "huggingface_hub", "ultralytics"])
        print("✅ Dependencies installed")
        return True
    except subprocess.CalledProcessError as e:
        print(f"❌ Failed to install dependencies: {e}")
        return False

def download_model_from_huggingface(model_name: str, output_dir: str):
    """Download model from Hugging Face."""
    try:
        from huggingface_hub import hf_hub_download
        
        print(f"Downloading model: {model_name}")
        print("This may take a few minutes...")
        
        # Try to download the model file
        # Common model file names
        possible_files = ["best.pt", "yolov8n.pt", "yolov8n-seg.pt", "model.pt", "weights/best.pt"]
        
        model_file = None
        for filename in possible_files:
            try:
                print(f"  Trying {filename}...")
                model_path = hf_hub_download(
                    repo_id=model_name,
                    filename=filename,
                    local_dir=output_dir,
                    local_dir_use_symlinks=False
                )
                model_file = model_path
                print(f"✅ Found model file: {filename}")
                break
            except Exception as e:
                continue
        
        if not model_file:
            # Try listing files in repo
            print("  Listing repository files...")
            from huggingface_hub import list_repo_files
            files = list_repo_files(repo_id=model_name)
            print(f"  Available files: {files}")
            
            # Look for .pt files
            pt_files = [f for f in files if f.endswith('.pt')]
            if pt_files:
                print(f"  Found .pt files: {pt_files}")
                model_path = hf_hub_download(
                    repo_id=model_name,
                    filename=pt_files[0],
                    local_dir=output_dir,
                    local_dir_use_symlinks=False
                )
                model_file = model_path
            else:
                raise Exception("No .pt model file found in repository")
        
        print(f"✅ Model downloaded to: {model_file}")
        return model_file
        
    except ImportError:
        print("❌ huggingface_hub not installed. Installing...")
        if install_dependencies():
            return download_model_from_huggingface(model_name, output_dir)
        else:
            return None
    except Exception as e:
        print(f"❌ Error downloading model: {e}")
        return None

def test_model(model_path: str):
    """Test if model loads correctly."""
    try:
        from ultralytics import YOLO
        print(f"Testing model: {model_path}")
        model = YOLO(model_path)
        print("✅ Model loaded successfully")
        
        # Check model info
        print(f"  Model type: {type(model.model)}")
        print(f"  Model classes: {len(model.names)} classes")
        print(f"  Classes: {list(model.names.values())[:10]}...")  # Show first 10
        
        # Check if license_plate is in classes
        class_names = [name.lower() for name in model.names.values()]
        if any('license' in name or 'plate' in name for name in class_names):
            print("✅ License plate class found in model!")
        else:
            print("⚠️  Warning: License plate class not found in model classes")
            print(f"   Available classes: {list(model.names.values())}")
        
        return True
    except Exception as e:
        print(f"❌ Error testing model: {e}")
        return False

def main():
    """Main function."""
    print("=" * 60)
    print("License Plate Detection Model Downloader")
    print("=" * 60)
    
    # Determine output directory
    workspace_root = Path.home() / "ugv_ws"
    models_dir = workspace_root / "src" / "amr_hardware" / "deep_learning_models"
    models_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"Output directory: {models_dir}")
    
    # Try different models in order of preference
    models_to_try = [
        "Koushim/yolov8-license-plate-detection",  # YOLOv8n, lightweight
        "RawanAlwadeya/YOLOCarAndPlateDetection",  # Detects cars and plates
        "morsetechlab/yolov11-license-plate-detection",  # YOLOv11
    ]
    
    downloaded_model = None
    
    for model_name in models_to_try:
        print(f"\n{'='*60}")
        print(f"Trying model: {model_name}")
        print(f"{'='*60}")
        
        model_file = download_model_from_huggingface(model_name, str(models_dir))
        
        if model_file and test_model(model_file):
            downloaded_model = model_file
            print(f"\n✅ Successfully downloaded and tested model!")
            print(f"   Model path: {model_file}")
            print(f"\n📝 Next steps:")
            print(f"   1. Update config.yaml to include 'license_plate' in interested_classes")
            print(f"   2. Update mission_controller.py to process license plate detections")
            print(f"   3. Update launch file to use this model if needed")
            break
        else:
            print(f"❌ Failed to download/test {model_name}, trying next...")
    
    if not downloaded_model:
        print("\n❌ Failed to download any model.")
        print("\nAlternative options:")
        print("1. Train your own model using the guide in ADDING_LICENSE_PLATE_DETECTION_TO_YOLO.md")
        print("2. Use a model from Roboflow Universe: https://universe.roboflow.com")
        print("3. Fine-tune yolov8n-seg.pt on your license plate dataset")
        return 1
    
    # Create symlink or copy to standard name
    license_plate_model_path = models_dir / "license_plate_model.pt"
    if downloaded_model != str(license_plate_model_path):
        import shutil
        if license_plate_model_path.exists():
            license_plate_model_path.unlink()
        shutil.copy2(downloaded_model, license_plate_model_path)
        print(f"\n✅ Model also saved as: {license_plate_model_path}")
    
    print("\n" + "=" * 60)
    print("Download complete!")
    print("=" * 60)
    return 0

if __name__ == "__main__":
    sys.exit(main())
