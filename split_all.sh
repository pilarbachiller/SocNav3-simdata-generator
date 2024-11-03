for f in `ls 2024*.json`; do
	python3 split_multigoal.py $f --videowidth 1400 --videoheight 1400 --leftcrop 180 --rightcrop 430 --topcrop 170 --bottomcrop 520  --rotate 59.5 --ffwd --novideo #2> /dev/null
done
