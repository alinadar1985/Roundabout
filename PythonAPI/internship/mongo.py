import pymongo
import urllib
import urllib.parse

def saveMongo(myCPM):
    client = pymongo.MongoClient("mongodb+srv://user1:user1@eurecom.lvt7r.mongodb.net/EURECOM?retryWrites=true&w=majority")
    try:
        #print(client.server_info())
        db = client.sample_weatherdata
        my_collection = db.aaa1 #  collection
        mydict = { "name": "Ali Nadar", "address": {"Rue":"2400 Route des dolines","Batiment":"Isaac Newton Residence", "Room": 130}, "University":"Eurecom" }
        x = my_collection.insert_one(myCPM)
        print("Saved in Mongo .......................................................")

        #for document in my_collection.find():
        #    print (document)
    except Exception:
        print("Unable to find data.")

