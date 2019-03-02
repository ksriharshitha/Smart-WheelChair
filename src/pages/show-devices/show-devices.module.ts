import { NgModule } from '@angular/core';
import { IonicPageModule } from 'ionic-angular';
import { ShowDevicesPage } from './show-devices';

@NgModule({
  declarations: [
    ShowDevicesPage,
  ],
  imports: [
    IonicPageModule.forChild(ShowDevicesPage),
  ],
})
export class ShowDevicesPageModule {}
