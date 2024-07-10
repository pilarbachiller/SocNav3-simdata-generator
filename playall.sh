for f in `ls ../*_trj.json`; do

    python3 view_data.py --videowidth 1600 --videoheight 1600 --rotate -120.5 --topcrop 590 --bottomcrop 180 --leftcrop 520 --rightcrop 230 $f

done