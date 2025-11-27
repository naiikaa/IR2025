#get all folder inside data and run data.py on each folder, wait till one is finished before starting the next one
#!/bin/bash
for folder in data/*/; do
    echo "Processing folder: $folder"
    /bin/python3 /home/npopkov/repos/IR2025/scen/data.py --car_dir "$folder"
done
