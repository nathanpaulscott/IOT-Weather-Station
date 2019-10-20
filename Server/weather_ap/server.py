from flask import Flask, url_for, escape, g, request, render_template, send_from_directory, jsonify
from flask_cors import CORS, cross_origin
import sqlite3
import json
from datetime import datetime as dt



def run_sql(db, sql, get_response=False, header=False):
	#sends an sql command to the db
	#note for creating tables
	#CREATE TABLE tabname (datetime_sec int8, datetime_str text, wspd_mean real, wdir_mean real, temp real, hum real, rep_cnt long, light_lev real, batt_lev real, rssi real, primary key(datetime_sec));
	#to drop
	#drop table if exists tabname;
	#to show schema
	#.schema tabname
	#####################################################
	response = 'ok'
	try:
	    with sqlite3.connect(db) as db_conn:
	        cur = db_conn.cursor()
	        cur.execute(sql)
		if get_response:
			response = cur.fetchall()
		
		if header:
			cols = [tuple([description[0] for description in cur.description])]
			return cols + response
		else:
			return response
	except Exception as e:
		return 'error: ' + str(e)
	


def data_processor(data, schema):
	#datetime_obj = dt.fromtimestamp(datetime_sec)
	#datetime_str = datetime_obj.strftime('%d/%m/%y %H:%M:%S')
	#datetime_sec = (datetime_obj-dt.datetime(1970,1,1)).total_seconds()

	#converts the incoming dict to a string for db update
	#check data format is ok
	if len(data) != len(schema_list): return None
	#note: dt.now() gives the current time, dt.utcfromtimestamp(0) gives the time at 0:0:0 1/1/1970
	dt_now = dt.now()
	datetime_str = dt_now.strftime('%d/%m/%y %H:%M:%S')
	secs_since_1970 = int((dt_now - dt.utcfromtimestamp(0)).total_seconds())
	data_str = '{0},"{1}"'.format(secs_since_1970, datetime_str)
	#print(data_str)
	for item in schema:
		if item not in data: return None
		data_str += "," + str(data[item])
	return data_str


app = Flask(__name__, static_url_path='')
CORS(app)
db_path = '/home/pi/weather_ap/weather.db'
schema_list = ['wspd_mean', 'wdir_mean', 'temp', 'hum', 'rep_cnt', 'light_lev', 'batt_lev', 'rssi'] 


@app.route('/')
def homepage():
    return send_from_directory('static', 'index.html')


#to send data from curl:
#curl --data "wspd_mean=11.2&wdir_mean=145.2&temp_mean=24.6&hum_mean=24.6&rep_cnt=42&light_lev=3.3&batt_lev=3.7&rssi=-44" http://192.168.20.4:5000/data
#this is for receiving data from the sensor
@app.route('/data', methods = ['POST'])
def rx_data():
	if request.method != 'POST': 
		return 'nok'
        
	#this will read in all the post data as a dict
        data = request.form
	print(data)
	#convert it to a string
	data_str = data_processor(data, schema_list)
	if data_str is None: return 'nok'
	#build the sql command and send it
	sql = 'insert into data values({0});'.format(data_str)
	response = run_sql(db_path, sql, get_response=False, header=False)    
	return response



#this is for viewing the db contents
#usage: http://ip:port/db/24  ... or use last for the most recent value
@app.route('/db/<hrs>')
def show_db(hrs):
	if hrs == 'last':
		sql = 'SELECT * FROM data ORDER BY datetime_sec DESC LIMIT 1;'
	else:
		#this will show the data for the last N hours
		secs_since_1970 = int((dt.now() - dt.utcfromtimestamp(0)).total_seconds())
		time_thd = secs_since_1970 - 3600*int(hrs)
		sql = 'select * from data where datetime_sec > {0};'.format(time_thd)
	response = run_sql(db_path, sql, get_response=True, header=True)    
	return render_template("sql_select.html", data = response);
        #return jsonify(response)
	


#this basically runs the flask webserver, which is just a development server, not a production server
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)



