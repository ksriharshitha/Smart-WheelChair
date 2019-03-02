import { Component } from '@angular/core';
import { IonicPage, NavController, NavParams, MenuController } from 'ionic-angular';

/**
 * Generated class for the SignupPage page.
 *
 * See https://ionicframework.com/docs/components/#navigation for more info on
 * Ionic pages and navigation.
 */

@IonicPage()
@Component({
  selector: 'page-signup',
  templateUrl: 'signup.html',
})
export class SignupPage {

  account=
	{
	    username: '',
	    first_name:'',
	    last_name:'',
	    email: '',
	    password: '',
	    phone_no:''
  };

  constructor(public menuCtrl: MenuController, public navCtrl: NavController, public navParams: NavParams) {
    
    this.menuCtrl.enable(false,'mainMenu');
  }

  ionViewDidLoad() {
    console.log('ionViewDidLoad SignupPage');
  }

  doSignup(){

    localStorage.setItem('username','sahaj');
    this.navCtrl.setRoot('HomePage');

  }

}
