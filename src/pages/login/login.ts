import { Component } from '@angular/core';
import { IonicPage, NavController, NavParams, ToastController, MenuController } from 'ionic-angular';

/**
 * Generated class for the LoginPage page.
 *
 * See https://ionicframework.com/docs/components/#navigation for more info on
 * Ionic pages and navigation.
 */

@IonicPage()
@Component({
  selector: 'page-login',
  templateUrl: 'login.html',
})
export class LoginPage {
  account= 
   {
    username: '',
    password: ''
  };

  
  
  constructor(public menuCtrl:MenuController, public navCtrl: NavController, public navParams: NavParams, public toastCtrl: ToastController) {
    
    this.menuCtrl.enable(false,'mainMenu');
  }

  ionViewDidLoad() {
    console.log('ionViewDidLoad LoginPage');
  }

  doLogin(){

    if(this.account.username == 'harshitha' && this.account.password == 'harshitha'){

      localStorage.setItem('username',this.account.username);
      this.navCtrl.setRoot('HomePage');
    }
    else{

      let toast = this.toastCtrl.create({
        message: "Invalid Credentials",
        duration: 1000,
        position: 'bottom'
        });
        toast.present();
    }

  }

}
