import { Component, ChangeDetectorRef } from '@angular/core';
import { IonicPage, NavController, NavParams, ToastController, AlertController, PopoverController, ModalController, Platform } from 'ionic-angular';
import { BluetoothSerial } from '@ionic-native/bluetooth-serial';
import { ShowDevicesPage } from '../show-devices/show-devices';
import { SpeechRecognition } from '@ionic-native/speech-recognition';
import { SMS } from '@ionic-native/sms';
import {
  GoogleMaps,
  GoogleMap,
  GoogleMapsEvent,
  Marker,
  GoogleMapsAnimation,
  MyLocation
 } from '@ionic-native/google-maps';

/**
 * Generated class for the ControlCenterPage page.
 *
 * See https://ionicframework.com/docs/components/#navigation for more info on
 * Ionic pages and navigation.
 */

@IonicPage()
@Component({
  selector: 'page-control-center',
  templateUrl: 'control-center.html',
})
export class ControlCenterPage {

  iconname = 'pause';
  bluetoothstate = 'connect';
  disconnectstate = 'No';
  unpairedDevices: any;
  pairedDevices: any;
  gettingDevices: Boolean;
  info: any;
  matches: String[];
  isRecording = false;
  speakstate = 'mic';
  ActionOK = false;
  mapReady: boolean = false;
  map: GoogleMap;

  constructor(public modalCtrl : ModalController,
    public navCtrl: NavController, 
    public navParams: NavParams, 
    public popoverCtrl: PopoverController, 
    public toastCtrl: ToastController, 
    private bluetoothSerial: BluetoothSerial, 
    public alertCtrl: AlertController,
    private speechRecognition: SpeechRecognition,
    private sms: SMS,
    private plt: Platform,
    private cd: ChangeDetectorRef) {
  }

  ionViewDidLoad() {
    console.log('ionViewDidLoad ControlCenterPage');
  }

  presentConfirm() {
    let alert = this.alertCtrl.create({
      title: 'Do you want to Disconnect?',
      message: 'Caution: Connected to a serial device!',
      buttons: [
        {
          text: 'No',
          role: 'cancel',
          handler: () => {
            console.log('No clicked');
            this.disconnectstate = 'No';
          }
        },
        {
          text: 'Yes',
          handler: () => {
            console.log('Yes clicked');
            this.disconnectstate = 'Yes';
          }
        }
      ]
    });
    alert.present();
  }

  startScanning() {
    
    this.pairedDevices = null;
    this.unpairedDevices = null;
    this.gettingDevices = true;
    this.bluetoothSerial.discoverUnpaired().then((success) => {
      this.unpairedDevices = success;
      this.gettingDevices = false;
      console.log('Scanning Complete');
      success.forEach(element => {
        //console.log('success');
      });
      let devicemodal=this.modalCtrl.create(ShowDevicesPage,{'paired': this.pairedDevices,'unpaired': this.unpairedDevices},{ showBackdrop:true,enableBackdropDismiss:true });
      devicemodal.present();
      devicemodal.onDidDismiss((data) => {

        if(data){
          this.bluetoothstate = data;
        }
        else{

          this.bluetoothSerial.disconnect();
          this.bluetoothstate = 'connect';
          let toast = this.toastCtrl.create({
            message: "Error while Connecting",
            duration: 2000,
            position: 'bottom'
            });
            toast.present();
        }
      });
    },
      (err) => {
        console.log(err);
      })

    this.bluetoothSerial.list().then((success) => {
      this.pairedDevices = success;
    },
      (err) => {
        console.log(err);
      })
  }
  
BluetoothEnable(bstate){

   this.bluetoothSerial.isEnabled().then(() => {
      console.log("bluetooth is enabled all G");
      this.BluetoothConnect(bstate);
    }, () => {
      console.log("bluetooth is not enabled trying to enable it");
          this.bluetoothSerial.enable().then(() => {
              console.log("bluetooth got enabled hurray");
              this.BluetoothConnect(bstate);
              }, () => {
                  console.log("user did not enabled");
                  let toast = this.toastCtrl.create({
                    message: "Error while connecting. Please Try Again.",
                    duration: 1000,
                    position: 'bottom'
                    });
                    toast.present();
                  });
     });
  }

  BluetoothConnect(bstate){

    if(bstate == 'disconnect' ){

      this.presentConfirm();
      if( this.disconnectstate == 'Yes' ){

        this.bluetoothstate = 'connect';
        this.bluetoothSerial.disconnect().then(() => {

            console.log("Bluetooth Disconnected");
          }, () => {

            let toast = this.toastCtrl.create({
              message: "Error while Disconnecting.",
              duration: 1000,
              position: 'bottom'
              });
              toast.present();
            });
      }          
    }
    else {

      this.disconnectstate = 'No';
      console.log('Trying to connect');
      this.startScanning(); 
    }
   
  }

  BluetoothAction(key){

    if(key == 'z'){

      if(this.iconname == 'play'){

        key = 'x';
      }
      this.changestate(this.iconname);
    }
    else{

      if(this.iconname == 'play'){

        this.changestate(this.iconname);
      }
    }
    this.bluetoothSerial.write(key).then((success) =>{

      console.log(key,' written');

    }, (fail) => {
      console.log('Error: ',key,' not written');
    });
  }
  
  changestate(name){

    if(name == 'pause'){

      this.iconname = 'play';
    }
    else{

      this.iconname = 'pause';
    }
  }

  showToast(message: string) {
    let toast = this.toastCtrl.create({
      message: message,
      duration: 2000,
      position: 'bottom'
    });

    toast.present(toast);
  }

  presentActionConfirm(action,key) {
    let alert = this.alertCtrl.create({
      title: 'Do you want to move '+action+'?',
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
            this.BluetoothAction(key);
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

        if("forward"==matches[i]){

          //this.presentActionConfirm("Forward",'w');
          this.BluetoothAction('w');
          i=-1;
          break;
        }
        else if("left"==matches[i]){

          //this.presentActionConfirm("Left",'a');
          this.BluetoothAction('a');
          i=-1;
          break;
        }
        else if("right"==matches[i]){

          //this.presentActionConfirm("Right",'d');
          this.BluetoothAction('d');
          i=-1;
          break;
        }
        else if("backward"==matches[i]){

          //this.presentActionConfirm("Backward",'s');
          this.BluetoothAction('s');
          i=-1;
          break;
        }
        else if("stop"==matches[i] || "start"==matches[i]){

          this.BluetoothAction('z');
          i=-1;
          break;
        } 
      }
      if(i!=-1){

        let toast = this.toastCtrl.create({
          message: "Action not recognised",
          duration: 2000,
          position: 'bottom'
          });
          toast.present();
      }
      /* if(this.ActionOK==false && i!=-1){

        let toast = this.toastCtrl.create({
          message: "Action not recognised",
          duration: 2000,
          position: 'bottom'
          });
          toast.present();
      } */
      this.speakstate='mic';
      this.cd.detectChanges();
    });
    this.isRecording = true;
    this.speakstate='mic-off';
  }
}
