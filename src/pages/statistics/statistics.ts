import { Component, ViewChild } from '@angular/core';
import { IonicPage, NavController, NavParams } from 'ionic-angular';

/**
 * Generated class for the StatisticsPage page.
 *
 * See https://ionicframework.com/docs/components/#navigation for more info on
 * Ionic pages and navigation.
 */

import { Chart } from 'chart.js';

@IonicPage()
@Component({
  selector: 'page-statistics',
  templateUrl: 'statistics.html',
})
export class StatisticsPage {

    @ViewChild('lineCanvas') lineCanvas;
    @ViewChild('densityCanvas') densityCanvas;
    @ViewChild('barCanvas') barCanvas;
 
    barChart: any;
    lineChart: any;
    horizontalbarChart: any;

  constructor(public navCtrl: NavController, public navParams: NavParams) {
  }

  ionViewDidLoad() {
    console.log('ionViewDidLoad StatisticsPage');
    this.lineChart = new Chart(this.lineCanvas.nativeElement, {
 
      type: 'line',
      data: {
          labels: ["2018-05-04", "2018-05-04", "2018-05-04", "2018-05-03", "2018-05-03", "2018-05-02", "2018-05-02"],
          datasets: [
              {
                  label: "Distance Travelled in m",
                  fill: false,
                  lineTension: 0.1,
                  backgroundColor: "rgba(75,192,192,0.4)",
                  borderColor: "rgba(75,192,192,1)",
                  borderCapStyle: 'butt',
                  borderDash: [],
                  borderDashOffset: 0.0,
                  borderJoinStyle: 'miter',
                  pointBorderColor: "rgba(75,192,192,1)",
                  pointBackgroundColor: "#fff",
                  pointBorderWidth: 1,
                  pointHoverRadius: 5,
                  pointHoverBackgroundColor: "rgba(75,192,192,1)",
                  pointHoverBorderColor: "rgba(220,220,220,1)",
                  pointHoverBorderWidth: 2,
                  pointRadius: 1,
                  pointHitRadius: 10,
                  data: [10, 11.6, 12.5, 13.8, 19.8, 21.7, 27],
                  spanGaps: false,
              }
          ]
      }

  });
  this.barChart = new Chart(this.barCanvas.nativeElement, {
 
    type: 'bar',
    data: {
        labels: ["2018-05-04", "2018-05-04", "2018-05-04", "2018-05-03", "2018-05-03", "2018-05-02", "2018-05-02"],
        datasets: [{
            label: 'unit bpm',
            data: [72, 78, 82, 76, 78, 85],
            backgroundColor: [
                'rgba(255, 99, 132, 0.2)',
                'rgba(54, 162, 235, 0.2)',
                'rgba(255, 206, 86, 0.2)',
                'rgba(75, 192, 192, 0.2)',
                'rgba(153, 102, 255, 0.2)',
                'rgba(255, 159, 64, 0.2)'
            ],
            borderColor: [
                'rgba(255,99,132,1)',
                'rgba(54, 162, 235, 1)',
                'rgba(255, 206, 86, 1)',
                'rgba(75, 192, 192, 1)',
                'rgba(153, 102, 255, 1)',
                'rgba(255, 159, 64, 1)'
            ],
            borderWidth: 1
        }]
    },
    options: {
        scales: {
            yAxes: [{
                ticks: {
                    beginAtZero:true
                }
            }]
        }

    }

});


this.horizontalbarChart = new Chart(this.densityCanvas.nativeElement, {
 
  type: 'horizontalBar',
  data: {
      labels: ["2018-05-02 17:45", "2018-05-02 22:30", "2018-05-03 18:30", "2018-05-03 20:00"],
      datasets: [{
          label: 'medicine status',
          data: [0,0,1,1,0,1,0,0,1],
          backgroundColor: [
              'rgba(255, 99, 132, 0.2)',
              'rgba(54, 169, 235, 0.2)',
              'rgba(255, 206, 86, 0.2)',
              'rgba(255, 192, 192, 0.2)',
              'rgba(153, 102, 255, 0.2)',
              'rgba(255, 159, 64, 0.2)'
          ],
          borderColor: [
              'rgba(255,99,132,1)',
              'rgba(54, 162, 235, 1)',
              'rgba(255, 206, 86, 1)',
              'rgba(75, 192, 192, 1)',
              'rgba(153, 102, 255, 1)',
              'rgba(255, 159, 64, 1)'
          ],
          borderWidth: 1
      }]
  },
  options: {
      scales: {
          yAxes: [{
              ticks: {
                  beginAtZero:true
              }
          }]
      }
      
  }

});

  }

}