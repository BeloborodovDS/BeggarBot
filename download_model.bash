mkdir -p data

if [ -f ./data/face_vino.xml ];
then
    echo "File face_vino.xml already exists, skipping"
else
    wget --no-check-certificate https://download.01.org/openvinotoolkit/2018_R4/open_model_zoo/face-detection-retail-0004/FP16/face-detection-retail-0004.xml -O ./data/face_vino.xml
fi

if [ -f ./data/face_vino.bin ];
then
    echo "File face_vino.bin already exists, skipping"
else
    wget --no-check-certificate https://download.01.org/openvinotoolkit/2018_R4/open_model_zoo/face-detection-retail-0004/FP16/face-detection-retail-0004.bin -O ./data/face_vino.bin
fi
