for f in `ls 2024*.json`; do
	python3 split_multigoal.py $f 
done
