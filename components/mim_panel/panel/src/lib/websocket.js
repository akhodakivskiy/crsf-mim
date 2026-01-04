export class WebSocketClient {
    constructor(url) {
        this.url = url;
        this.ws = null;
        this.onMessage = null;
        this.onConnected = null;
        this.onDisconnected = null;
    }

    connect() {
        if (this.ws) {
            this.ws.close();
        }

        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws';
        const wsUrl = `${protocol}//${window.location.host}/ws`;
        
        this.ws = new WebSocket(wsUrl);
        
        this.ws.onopen = () => {
            console.log('WebSocket connected');
            if (this.onConnected) {
                this.onConnected();
            }
        };

        this.ws.onmessage = (event) => {
            try {
                const message = JSON.parse(event.data);
                if (this.onMessage) {
                    this.onMessage(message);
                }
            } catch (error) {
                console.error('Failed to parse message:', error);
            }
        };

        this.ws.onclose = () => {
            console.log('WebSocket disconnected');
            this.ws = null;
            if (this.onDisconnected) {
                this.onDisconnected();
            }
        };

        this.ws.onerror = (error) => {
            console.error('WebSocket error:', error);
        };
    }

    disconnect() {
        if (this.ws) {
            this.ws.close();
            this.ws = null;
        }
    }

    send(message) {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify(message));
        }
    }
}

export function useWebSocket(url, callbacks) {
    const client = new WebSocketClient(url);
    
    if (callbacks.onMessage) {
        client.onMessage = callbacks.onMessage;
    }
    
    if (callbacks.onConnected) {
        client.onConnected = callbacks.onConnected;
    }
    
    if (callbacks.onDisconnected) {
        client.onDisconnected = callbacks.onDisconnected;
    }
    
    return client;
}