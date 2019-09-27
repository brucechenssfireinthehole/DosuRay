import json
import pprint

data = json.load(open('/home/dosu/personal/code/beta/study_code/geo_localization/DosuRay/data/rows.json','rb'))
pprint.pprint(data,depth=1)
