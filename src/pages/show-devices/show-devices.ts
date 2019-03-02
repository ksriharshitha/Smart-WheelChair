import { Component,Renderer } from '@angular/core';
import { IonicPage, NavController, NavParams, ViewController, AlertController } from 'ionic-angular';
import { BluetoothSerial } from '@ionic-native/bluetooth-serial';
/**
 * Generated class for the ShowDevicesPage page.
 *
 * See https://ionicframework.com/docs/components/#navigation for more info on
 * Ionic pages and navigation.
 */

@IonicPage()
@Component({
  selector: 'page-show-devices',
  templateUrl: 'show-devices.html',
})
export class ShowDevicesPage {

  pairedDevices: any;
  unpairedDevices: any;

  constructor(public navCtrl: NavController, public navParams: NavParams,public renderer: Renderer, public viewCtrl:ViewController, private bluetoothSerial: BluetoothSerial, public alertCtrl: AlertController) {
  
    this.renderer.setElementClass(viewCtrl.pageRef().nativeElement,'page-show-devices',true);
  }

  ionViewDidLoad() {
    console.log('ionViewDidLoad ShowDevicesPage');
    this.pairedDevices = this.navParams.get('paired');
    this.unpairedDevices = this.navParams.get('unpaired');
  }

  success = (data) => {
    this.viewCtrl.dismiss('disconnect');
    alert(data)  
  };
  fail = (error) => {
    
    this.viewCtrl.dismiss('connect');
    alert(error)
  };

  selectDevice(address: any) {

    let alert = this.alertCtrl.create({
      title: 'Connect',
      message: 'Do you want to connect?',
      buttons: [
        {
          text: 'Cancel',
          role: 'cancel',
          handler: () => {
            console.log('Cancel clicked');
          }
        },
        {
          text: 'Connect',
          handler: () => {
            this.bluetoothSerial.connect(address).subscribe(this.success, this.fail);
          }
        }
      ]
  });
    alert.present();

  }
}
