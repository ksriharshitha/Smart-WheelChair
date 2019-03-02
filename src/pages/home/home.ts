import { Component, ChangeDetectorRef } from '@angular/core';
import { IonicPage, NavController, Platform, NavParams, MenuController, ToastController, AlertController} from 'ionic-angular';
import {
  GoogleMaps,
  GoogleMap,
  GoogleMapsEvent,
  Marker,
  GoogleMapsAnimation,
  MyLocation
 } from '@ionic-native/google-maps';
 import { SMS } from '@ionic-native/sms';
 import { DeviceOrientation, DeviceOrientationCompassHeading } from '@ionic-native/device-orientation';
 import { SpeechRecognition } from '@ionic-native/speech-recognition';

/**
 * Generated class for the HomePage page.
 *
 * See https://ionicframework.com/docs/components/#navigation for more info on
 * Ionic pages and navigation.
 */

@IonicPage()
@Component({
  selector: 'page-home',
  templateUrl: 'home.html',
})
export class HomePage {

  mapReady: boolean = false;
  map: GoogleMap;
  matches: String[];
  isRecording = false;
  speakstate = 'mic';
  ActionOK = false;

  constructor(public menuCtrl:MenuController, 
    public navCtrl: NavController, 
    public navParams: NavParams, 
    public toastCtrl: ToastController,
    private sms: SMS,
    public alertCtrl: AlertController,
    private speechRecognition: SpeechRecognition,
    private plt: Platform,
    private deviceOrientation: DeviceOrientation,
    private cd: ChangeDetectorRef) {
    
    this.menuCtrl.enable(true,'mainMenu');
  }

  ionViewDidLoad() {
    console.log('ionViewDidLoad HomePage');
    this.loadMap();
  }

  loadMap(){

     // Create a map after the view is loaded.
    // (platform is already ready in app.component.ts)
    this.map = GoogleMaps.create('map_canvas', {
      camera: {
        target: {
          lat: 13.5495461,
          lng: 79.99959
        },
        zoom: 18,
        tilt: 30
      }
      
    });

    // Wait the maps plugin is ready until the MAP_READY event
    this.map.one(GoogleMapsEvent.MAP_READY).then(() => {
      this.mapReady = true;
      this.map.setMyLocationEnabled(true);
    });
  }
  onButtonClick() {
    if (!this.mapReady) {
      this.showToast('map is not ready yet. Please try again.');
      return;
    }
    this.map.clear();

    // Get the location of you
    this.map.getMyLocation()
      .then((location: MyLocation) => {
        console.log(JSON.stringify(location, null ,2));

        // Move the map camera to the location with animation
        return this.map.animateCamera({
          target: location.latLng,
          zoom: 17,
          tilt: 30
        }).then(() => {
          // add a marker
          return this.map.addMarker({
            title: 'Your Location\nLatitude: '+location.latLng.lat+'\nLongitude: '+location.latLng.lng,
            snippet: '',
            position: location.latLng,
            animation: GoogleMapsAnimation.BOUNCE
          });
        })
      }).then((marker: Marker) => {
        // show the infoWindow
        marker.showInfoWindow();

        // If clicked it, display the alert
        marker.on(GoogleMapsEvent.MARKER_CLICK).subscribe(() => {
        });
      });
  }

  showToast(message: string) {
    let toast = this.toastCtrl.create({
      message: message,
      duration: 2000,
      position: 'bottom'
    });

    toast.present(toast);
  }
  sendHelpSms(){

    this.sms.hasPermission().then((hasPermission: boolean) => {

      if(!hasPermission){
        
        this.showToast("App does not have SMS permission");
      }
    });
    let options = {
      replaceLineBreaks: true,
    }
    this.map.getMyLocation().then((location: MyLocation) => {
    
      this.sms.send('+919908767777','Please help me. My coordinates are:\nLatitude: '+location.latLng.lat+',\nLongitude: '+location.latLng.lng,options).then((success) => {

        if(success){

          let alert = this.alertCtrl.create({
            title: 'Caution!',
            subTitle: 'Help sms sent.',
            buttons: ['OK']
          });
          alert.present();
        }
      });
    });
  }

  presentActionConfirm(action) {
    let alert = this.alertCtrl.create({
      title: 'Do you want to get '+action+'?',
      message: '',
      buttons: [
        {
          text: 'No',
          role: 'cancel',
          handler: () => {
            console.log('No clicked');
            this.ActionOK = false;
          }
        },
        {
          text: 'Yes',
          handler: () => {
            console.log('Yes clicked');
            this.ActionOK = true;
            if(action=='Location'){

              this.onButtonClick();
            }
            else{

              this.sendHelpSms();
            }
          }
        }
      ]
    });
    alert.present();
  }
  isIos() {
    return this.plt.is('ios');
  }
 
  stopListening() {
    this.speechRecognition.stopListening().then(() => {
      this.isRecording = false;
      this.speakstate='mic';
    });
  }

  startListening() {
    this.ActionOK = false;
    let options = {
      language: 'en-US'
    }
    this.speechRecognition.hasPermission()
      .then((hasPermission: boolean) => {
        if (!hasPermission) {
          this.speechRecognition.requestPermission();
        }
    });
    this.speechRecognition.startListening().subscribe(matches => {
      this.matches = matches;
      var i;
      for(i in matches){

        if("my location"==matches[i]){

          //this.presentActionConfirm("Location");
          this.onButtonClick();
          i=-1;
          break;
        }
        else if("help"==matches[i]){

          //this.presentActionConfirm("Help");
          this.sendHelpSms();
          i=-1;
          break;
        }
      }
      if(i!=-1){

        this.showToast("Action not recognised")
      }
      /* if(this.ActionOK==false && i!=-1){

        this.showToast("Action not recognised")
      } */
      this.speakstate='mic';
      this.cd.detectChanges();
    });
    this.isRecording = true;
    this.speakstate='mic-off';
  }
}
