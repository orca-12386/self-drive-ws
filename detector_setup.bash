# Create a Python virtual environment named YOLO using Python 3.10
python3.10 -m venv YOLO

# Activate the virtual environment
source YOLO/bin/activate

# Upgrade pip and install dependencies from requirements file
pip install --upgrade pip
pip install -r detector_req.txt

# Set environment variables
export PYTHONPATH=$PYTHONPATH:'YOLO/lib/python3.10/site-packages'
export PYTHON_EXECUTABLE=$(which python3)

# Copy trained models to the correct location
cp -r 'detective/detective/Trained_models' 'install/detective/share/detective/'

