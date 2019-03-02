import { Component, ViewChild } from '@angular/core';
import { Nav, Platform } from 'ionic-angular';
import { StatusBar } from '@ionic-native/status-bar';
import { SplashScreen } from '@ionic-native/splash-screen';
import { HomePage } from '../pages/home/home';
import { WelcomePage } from '../pages/welcome/welcome';
import { ControlCenterPage } from '../pages/control-center/control-center';
import { StatisticsPage } from '../pages/statistics/statistics';

@Component({
  templateUrl: 'app.html'
})
export class MyApp {
  @ViewChild(Nav) nav: Nav;

  rootPage: any = WelcomePage;

  pages: Array<{title: string,icon: string, component: any}>;

  constructor(public platform: Platform, public statusBar: StatusBar, public splashScreen: SplashScreen) {
    this.initializeApp();

    // used for an example of ngFor and navigation
    this.pages = [
      { title: 'Home', icon: 'home', component: HomePage },
      { title: 'Control Center', icon: 'walk', component: ControlCenterPage },
      { title: 'Statistics', icon: 'pulse', component: StatisticsPage },
      { title: 'Logout', icon: 'ios-log-out', component: WelcomePage},
    ];

  }

  initializeApp() {
    if(localStorage.getItem('username') == ''){
      this.rootPage = WelcomePage;
    }
    else{
      this.rootPage = HomePage;
    }
    this.platform.ready().then(() => {
      // Okay, so the platform is ready and our plugins are available.
      // Here you can do any higher level native things you might need.
      this.statusBar.styleDefault();
      this.splashScreen.hide();
    });
  }

  openPage(page) {
    if( page.component == WelcomePage){

      localStorage.setItem('username','');
    }
    // Reset the content nav to have just this page
    // we wouldn't want the back button to show in this scenario
    this.nav.setRoot(page.component);
  }
}
