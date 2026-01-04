import { mount } from 'svelte'
import App from './App.svelte'

// Ensure DOM is ready
const target = document.getElementById('app')

if (!target) {
  throw new Error('Target element #app not found')
}

const app = mount(App, { target })

export default app
